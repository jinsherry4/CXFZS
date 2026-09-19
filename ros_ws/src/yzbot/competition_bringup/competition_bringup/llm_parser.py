# -*- coding: utf-8 -*-
"""LLM 解题节点：随机题目 → 云端大模型 → 结构化抓取指令 [red,4,A;blue,1,C]。

输入：
    /mission/question (std_msgs/String)  出题器输出的题目文本
输出：
    /mission/command   (std_msgs/String) JSON {"question":..., "items":[...]}
    /mission/llm_status (std_msgs/String) 解析状态（面板显示：解析中/成功/失败）

鲁棒性：
    - 调用超时 20s，失败重试（网络抖动）
    - 输出归一化：全角→半角、中文颜色→英文、大小写、空白
    - 校验：颜色∈{red,blue}、区域∈{A,B,C}、数量 1..5；同色同区去重合并
    - 解析失败自动用"仅解题"提示词重试一次
参数：
    api_base  OpenAI 兼容接口地址（默认 DeepSeek）
    api_key   云端 API Key（为空时回退环境变量 DEEPSEEK_API_KEY；再为空进入 MOCK）
    model     模型名（deepseek-chat / glm-4-flash 等）
    mapping   比赛公布的映射规则文本（拼进提示词）
"""
import json
import os
import re
import urllib.request

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

VALID_COLORS = {'red', 'blue'}
VALID_ZONES = {'A', 'B', 'C'}
COLOR_ALIAS = {'red': 'red', '红色': 'red', '红': 'red', 'r': 'red',
               'blue': 'blue', '蓝色': 'blue', '蓝': 'blue', 'b': 'blue'}
MAX_COUNT = 5          # 每种颜色场景中最多 5 个方块
FULLWIDTH = {ord(f): ord(t) for f, t in zip(
    '０１２３４５６７８９，；：（）［］', '0123456789,;:()[]')}

SOLVE_PROMPT = (
    '你解题要非常仔细。这类题是"容量分组"应用题：x、y 是柜子/店铺的个数'
    '（组数），不是物品总数。解法固定两步：①把每类物品的需求逐人累加得'
    '总数；②总数 ÷ 每柜容量，向上取整（不足整柜也要占一个柜子）。\n'
    '先一步步推理（计算过程要核对），然后严格按照格式输出最终指令。\n'
    '输出要求：最后一行必须单独输出形如 [red,4,A;blue,1,C] 的文本：'
    '方括号包裹，多条指令用分号分隔，每条为 颜色(英文red或blue),'
    '数量(整数),区域(大写字母A/B/C)。最后一行之前不要出现任何方括号。\n\n'
    '示例：\n'
    '题目：每个衣柜有5件衣物；有T恤、裤子两种衣柜；甲需要3件T恤,1条裤子；'
    '乙需要8件T恤；丙需要4条裤子。设T恤衣柜数量为x，裤子衣柜数量为y，'
    '计算x、y分别为多少。\n'
    '推理：T恤总数=3+8=11件，11÷5=2.2，向上取整 x=3；'
    '裤子总数=1+4=5件，5÷5=1，y=1。\n'
    '映射规则：x代表红色，y代表蓝色；数量大于等于3的去A区，其他去C区。'
    '红色数量3，3>=3去A区；蓝色数量1，1<3去C区。\n'
    '最终指令：\n'
    '[red,3,A;blue,1,C]\n\n'
    '映射规则：{mapping}\n题目：{question}'
)

# 裁判直述任务句式（评分流程一：裁判发布搬运物体颜色与放置区域任务）：
# "搬运2个红色包裹到A区，3个蓝色包裹至C区" —— 无需映射计算，直接提取。
TASK_PROMPT = (
    '从裁判任务指令中提取搬运任务。只输出一行形如 [red,2,A;blue,3,C] 的指令：'
    '方括号包裹，多条用分号分隔，每条为 颜色(red/blue),数量(整数),区域(A/B/C)。'
    '不要推理，不要输出其他任何内容。\n'
    '示例：\n任务：搬运2个红色包裹到A区，3个蓝色包裹至C区\n'
    '输出：[red,2,A;blue,3,C]\n\n任务：红色1个到A区，蓝色2个去C区\n'
    '输出：[red,1,A;blue,2,C]\n\n任务：{question}\n输出：'
)

RETRY_PROMPT = (
    '只输出一行最终答案，格式为 [red,4,A;blue,1,C]（颜色red/blue,数量整数,区域A/B/C，分号分隔）。'
    '不要输出其他任何内容。\n映射规则：{mapping}\n题目：{question}'
)


class LlmParser(Node):

    def __init__(self):
        super().__init__('llm_parser')
        self.declare_parameter('api_base', 'https://api.deepseek.com/v1')
        self.declare_parameter('api_key', '')
        self.declare_parameter('model', 'deepseek-chat')
        self.declare_parameter(
            'mapping', 'x代表红色,y代表蓝色; 数量大于等于3的去A区, 其他去C区')
        self.declare_parameter('timeout_sec', 20.0)
        self.declare_parameter('max_retries', 2)
        self.pub = self.create_publisher(String, '/mission/command', 10)
        self.pub_status = self.create_publisher(String, '/mission/llm_status', 10)
        self.create_subscription(String, '/mission/question', self._on_question, 10)
        self._busy = False
        # r14 复盘：question_bridge 为保证 DDS 送达连发 10 次，_busy 只挡
        # 并发不挡执行器队列重入——每条都真调一次 API（10 次调用）。缓存
        # 上一题：重复题只重发缓存指令，不重复调 API。
        self._last_question = None
        self._last_items = None
        self.get_logger().info('llm_parser 就绪（等待 /mission/question）')

    def _status(self, text):
        m = String()
        m.data = text
        self.pub_status.publish(m)
        self.get_logger().info(f'[解析] {text}')

    def _on_question(self, msg: String):
        if self._busy:
            self.get_logger().warn('上一题仍在解析，忽略新题目')
            return
        question = msg.data.strip()
        if not question:
            return
        if question == self._last_question and self._last_items is not None:
            self._publish(question, self._last_items)
            self.get_logger().info('重复题目：重发缓存指令（不重复调 API）')
            return
        self._busy = True
        try:
            self._solve(question)
        finally:
            self._busy = False

    def _solve(self, question):
        api_key = (self.get_parameter('api_key').value
                   or os.environ.get('DEEPSEEK_API_KEY', '')).strip()
        if not api_key:
            self._status('未配置 api_key，使用 MOCK 指令（仅联调用）')
            items = self._mock_solve()
            self._publish(question, items)
            return
        self._status('大模型解析中…')
        answer = None
        items = []
        # 评分流程一（裁判直述任务）：含颜色+区且非数学题 → 用 TASK_PROMPT
        # 放宽：含颜色 + 任意区域表达（区/A/B/C）即视为裁判直述任务，
        # 支持 "2红到A"、"蓝2去C" 等极简写法。
        is_direct_task = (re.search(r'红|蓝|red|blue', question)
                          and re.search(r'区|[ABCabc]', question)
                          and not re.search(r'计算|x=|y=|分别为多少', question))
        prompts = [TASK_PROMPT if is_direct_task else SOLVE_PROMPT, RETRY_PROMPT]
        retries = int(self.get_parameter('max_retries').value)
        for attempt in range(max(1, retries + 1)):
            prompt_tpl = prompts[0] if attempt == 0 else prompts[-1]
            try:
                answer = self._call_llm(prompt_tpl, question, api_key)
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(f'LLM 调用失败(第{attempt + 1}次): {e}')
                continue
            self.get_logger().info(f'LLM 原始回答: {answer!r}')
            items = self._parse_answer(answer)
            if items:
                break
            self._status(f'第 {attempt + 1} 次输出无法解析，重试')
        if not items:
            self._status('解析失败：未能从大模型回答中获得有效指令，使用回退指令（各1个到C区）')
            items = [{'color': 'red', 'count': 1, 'zone': 'C'},
                     {'color': 'blue', 'count': 1, 'zone': 'C'}]
        self._publish(question, items)

    def _publish(self, question, items):
        self._last_question = question
        self._last_items = items
        out = String()
        out.data = json.dumps({'question': question, 'items': items},
                              ensure_ascii=False)
        self.pub.publish(out)
        brief = ';'.join(f"{it['color']},{it['count']},{it['zone']}" for it in items)
        self._status(f'解析成功: [{brief}]')

    # ---------- LLM ----------
    def _call_llm(self, prompt_tpl, question, api_key):
        base = self.get_parameter('api_base').value.rstrip('/')
        model = self.get_parameter('model').value
        mapping = self.get_parameter('mapping').value
        timeout = float(self.get_parameter('timeout_sec').value)
        prompt = prompt_tpl.format(mapping=mapping, question=question)
        body = json.dumps({
            'model': model,
            'messages': [{'role': 'user', 'content': prompt}],
            'temperature': 0.0,
            'max_tokens': 512,
        }).encode('utf-8')
        req = urllib.request.Request(
            base + '/chat/completions', data=body,
            headers={'Content-Type': 'application/json',
                     'Authorization': f'Bearer {api_key}'})
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            data = json.loads(resp.read().decode('utf-8'))
        return data['choices'][0]['message']['content']

    # ---------- 解析 ----------
    @classmethod
    def _parse_answer(cls, text):
        if not text:
            return []
        text = text.translate(FULLWIDTH)
        matches = re.findall(r'\[([^\[\]]+)\]', text)
        if not matches:
            return []
        items = {}
        for seg_text in matches[-1].split(';'):
            seg = [s.strip() for s in seg_text.split(',')]
            if len(seg) != 3:
                continue
            color = COLOR_ALIAS.get(seg[0].lower())
            zone = seg[2].strip().upper()
            try:
                count = int(float(seg[1]))
            except ValueError:
                continue
            if color is None or zone not in VALID_ZONES:
                continue
            count = max(1, min(MAX_COUNT, count))
            key = (color, zone)
            items[key] = items.get(key, 0) + count
        merged = []
        for (color, zone), count in items.items():
            # 同色多区时保留数量大的指令，避免超抓
            merged.append({'color': color, 'count': min(MAX_COUNT, count), 'zone': zone})
        by_color = {}
        for it in merged:
            cur = by_color.get(it['color'])
            if cur is None or it['count'] > cur['count']:
                by_color[it['color']] = it
        return list(by_color.values())

    @staticmethod
    def _mock_solve():
        """无 Key 时的示例指令：1红去A、2蓝去C（验证全链路用）。"""
        return [{'color': 'red', 'count': 1, 'zone': 'A'},
                {'color': 'blue', 'count': 2, 'zone': 'C'}]


def main(args=None):
    rclpy.init(args=args)
    node = LlmParser()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

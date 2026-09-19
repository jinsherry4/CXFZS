# -*- coding: utf-8 -*-
"""裁判命令节点（评分细则对齐版）。

官方流程（评分细则·流程一）：裁判发布搬运任务（自然语言）→ 大模型解析 →
解析结果显示到可视化面板 → 机器人按大模型输出执行。

链路：
  /referee/text (裁判原话)
    ├─ 有 API: 原话转发 /mission/question → llm_parser(DeepSeek TASK_PROMPT)
    │          → /mission/command → 回执"大模型解析成功: ..."(评分项2)
    └─ 无 API: 规则解析兜底 → 直接 /mission/command（离线演示模式）
"""
import json
import os
import re

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

CN_NUM = {'零': 0, '一': 1, '两': 2, '二': 2, '三': 3, '四': 4, '五': 5,
          '六': 6, '七': 7, '八': 8, '九': 9, '十': 10}


def _num_from(text, default=1):
    m = re.search(r'(\d+)', text)
    if m:
        return int(m.group(1))
    for ch in text:
        if ch in CN_NUM:
            return CN_NUM[ch]
    return default


def rule_parse(text):
    """离线兜底解析：按文本出现顺序提取 (color, count, zone)。

    兼容两种语序："1个红"（数量在前）与 "红色1个"（颜色在前），
    按匹配位置排序后与提取到的区域顺序一一配比。
    """
    t = text.replace(' ', '').lower()
    picked = []  # (pos, color, count)
    # 数量在前：1个红 / 2块蓝
    for m in re.finditer(
            r'([x×*]?\d+|[零一二两三四五六七八九十])(个|只|块|件)?(红|蓝|red|blue)',
            t):
        picked.append((m.start(),
                       'red' if m.group(3) in ('红', 'red') else 'blue',
                       _num_from(m.group(1), 1)))
    # 颜色在前：红色1个 / 蓝的2只（跳过与前类重叠的"红...1"模式段）
    for m in re.finditer(
            r'(红|蓝|red|blue)[^A-Za-z0-9]{0,6}?([x×*]?\d+|[零一二两三四五六七八九十])',
            t):
        color = m.group(1)
        if color in ('红', 'red'):
            color = 'red'
        else:
            color = 'blue'
        picked.append((m.start(), color, _num_from(m.group(2), 1)))
    if not picked:
        return None
    # 同一位置附近去重（如 "红色1个" 两条规则都命中同一文本段）
    picked.sort(key=lambda x: x[0])
    merged = []
    for pos, color, count in picked:
        if merged and pos - merged[-1][0] < 4:
            continue
        merged.append((pos, color, count))
    items = [{'color': c, 'count': n, 'zone': None} for _, c, n in merged]
    zones_in_text = [z.upper() for z in
                     re.findall(r'[到放去至into]+\s*([abc])\s*区?', t)]
    for i, it in enumerate(items):
        if zones_in_text:
            it['zone'] = zones_in_text[i % len(zones_in_text)]
        else:
            it['zone'] = 'A'
    return items



# 裁判快捷任务：Foxglove Raw Messages 里只需发布 "任务3"（或 3），
# 免填完整句式——演示/比赛时降低输入门槛。
PRESETS = {
    '1': [('red', 1, 'A'), ('blue', 2, 'C')],
    '2': [('blue', 2, 'C'), ('red', 1, 'B')],
    '3': [('red', 3, 'B')],
    '4': [('blue', 2, 'A')],
}

class RefereeNode(Node):

    def __init__(self):
        super().__init__('referee_node')
        self.declare_parameter('api_key', '')
        self.pub_cmd = self.create_publisher(String, '/mission/command', 10)
        self.pub_ack = self.create_publisher(String, '/referee/ack', 10)
        self.pub_ws = self.create_publisher(String, '/mission/work_state', 10)
        self.pub_q = self.create_publisher(String, '/mission/question', 10)
        # llm_parser 解析完成（/mission/command）→ 匹配裁判原话回执
        self.create_subscription(String, '/mission/command', self._on_parsed, 10)
        self.create_subscription(String, '/referee/text', self.on_text, 10)
        self._last_text = None
        api = (self.get_parameter('api_key').value
               or os.environ.get('DEEPSEEK_API_KEY', '')).strip()
        self._mode = 'llm' if api else 'rule'
        self.get_logger().info(
            f'referee_node 就绪（{"大模型解析" if self._mode == "llm" else "离线规则"}模式）:'
            ' 发 /referee/text')

    def _ack(self, ok, text, detail):
        msg = String()
        msg.data = json.dumps({'ok': ok, 'input': text, 'detail': detail},
                              ensure_ascii=False)
        self.pub_ack.publish(msg)
        ws = String()
        ws.data = f'裁判指令: {detail}'
        self.pub_ws.publish(ws)
        self.get_logger().info(f'[裁判回执] ok={ok} {detail}')

    def _dispatch(self, items, raw, via):
        m = String()
        m.data = json.dumps({'question': raw, 'items': items}, ensure_ascii=False)
        self.pub_cmd.publish(m)
        seq = '；'.join(f"{it['color']}×{it['count']}→{it['zone']}区"
                        for it in items)
        self._ack(True, raw, f'{via}: {seq}（开始执行）')

    def _on_parsed(self, msg: String):
        """llm_parser 完成解析：匹配最近裁判文本后回执（评分项2证据）。"""
        try:
            d = json.loads(msg.data)
        except Exception:
            return
        q = d.get('question', '')
        if self._last_text and q == self._last_text:
            items = d.get('items', [])
            seq = '；'.join(f"{it.get('color')}×{it.get('count')}→{it.get('zone')}区"
                            for it in items)
            self._ack(True, self._last_text,
                      f'大模型解析成功: {seq}（开始执行）')
            self._last_text = None

    def on_text(self, msg: String):
        text = (msg.data or '').strip()
        if not text:
            return
        # 快捷指令：任务1..4 / 1..4
        m = re.fullmatch(r'(?:任务)?([1-4一二三四])', text)
        if m:
            c = m.group(1)
            key = {'一': '1', '二': '2', '三': '3', '四': '4'}.get(c, c)
            items = [{'color': c2, 'count': n, 'zone': z}
                     for c2, n, z in PRESETS[key]]
            self._dispatch(items, f'快捷任务{key}', '快捷指令')
            return
        if self._mode == 'llm':
            # 官方链路：裁判原话 → /mission/question → llm_parser 大模型解析
            self._last_text = text
            m = String()
            m.data = text
            self.pub_q.publish(m)
            self._ack(True, text, '已提交大模型解析（结果将显示于任务信息面板）')
            return
        # 离线兜底
        items = rule_parse(text)
        if items is None:
            self._ack(False, text, '无法解析出抓取项（需包含颜色+数量）')
            return
        self._dispatch(items, text, '离线规则解析')


def main(args=None):
    rclpy.init(args=args)
    n = RefereeNode()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

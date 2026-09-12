# -*- coding: utf-8 -*-
"""向世界文件静态几何注入软接触参数(防弹飞)，带完整性校验。

只替换 <collision> 内的空 <contact><ode/></contact>；
跳过 ground_plane(保持地面刚性，避免机器人下陷)；
写临时文件→校验→原子替换，任何异常不落盘。
"""
import re
import shutil
import sys
import tempfile
import xml.etree.ElementTree as ET

SRC = '/home/ros/dev_ws/src/yzbot/mybot_description/worlds/offic_room.world'
BAK = SRC + '.bak7'

SOFT = ('<contact><ode><kp>2e5</kp><kd>100</kd>'
        '<max_vel>0</max_vel><min_depth>0.02</min_depth></ode></contact>')

with open(SRC, 'r', encoding='utf-8') as f:
    text = f.read()
with open(BAK, 'r', encoding='utf-8') as f:
    base = f.read()


def model_names(xml_text):
    return re.findall(r"<model name='([^']+)'", xml_text)


base_models = model_names(base)
src_models = model_names(text)
if sorted(base_models) != sorted(src_models):
    print(f'FATAL: 当前文件模型集合与 bak7 不一致: '
          f'{len(base_models)} vs {len(src_models)}')
    sys.exit(1)

# 空接触块（允许中间有空白），仅在 <collision> 内替换
pat = re.compile(r'<contact>\s*<ode/>\s*</contact>')

# 逐段处理：按 <model ...> ... </model> 切分，跳过 ground_plane
out, pos, replaced = [], 0, 0
for m in re.finditer(r"<model name='([^']+)'>.*?</model>", text, re.S):
    out.append(text[pos:m.start()])
    block = m.group(0)
    if m.group(1) != 'ground_plane':
        block, n = pat.subn(SOFT, block)
        replaced += n
    out.append(block)
    pos = m.end()
out.append(text[pos:])
new_text = ''.join(out)

if replaced == 0:
    # 幂等：已经是软接触则直接通过
    if new_text.count('kp>2e5') == 31 or 'kp>2e5' in text:
        print('already soft, skip')
        sys.exit(0)
    print('FATAL: 未找到可替换的空接触块')
    sys.exit(1)

# 校验1: XML 可解析
try:
    ET.fromstring(new_text)
except ET.ParseError as e:
    print(f'FATAL: 修改后 XML 无法解析: {e}')
    sys.exit(1)

# 校验2: 模型集合不变
if sorted(model_names(new_text)) != sorted(base_models):
    print('FATAL: 修改后模型集合变化')
    sys.exit(1)

# 校验3: 将两侧所有接触块归一化为占位符后，文件必须完全一致
# （证明除接触块外零改动，含空白/换行）
def norm(t):
    t = re.sub(r'<contact>\s*<ode/>\s*</contact>', '<CONTACT/>', t, flags=re.S)
    t = re.sub(r'<contact><ode>.*?</ode></contact>', '<CONTACT/>', t, flags=re.S)
    return t

if norm(new_text) != norm(base):
    print('FATAL: 接触块以外存在差异')
    sys.exit(1)

tmp = SRC + '.tmp_soft'
with open(tmp, 'w', encoding='utf-8') as f:
    f.write(new_text)
shutil.move(tmp, SRC)
print(f'soft-contact injected: {replaced}, '
      f'models={len(model_names(new_text))}, verified OK')

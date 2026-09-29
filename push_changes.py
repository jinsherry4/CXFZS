#!/usr/bin/env python3
"""增量推送：读取最近一次提交的变更文件清单 → 调 api_push 推送（规避中文路径 shell 传参问题）。"""
import subprocess
import sys
import os
from pathlib import Path

os.environ.setdefault(
    "API_PUSH_MSG",
    "同步 09-22~09-29: systemd服务化+r51-r63导航优化+3D简化可视化+验证证据包")

ROOT = Path(__file__).parent
sys.path.insert(0, str(ROOT))
import api_push  # noqa: E402

out = subprocess.run(
    ["git", "-c", "core.quotepath=false", "diff", "--name-only", "HEAD~1", "HEAD"],
    capture_output=True, text=True, encoding="utf-8", cwd=ROOT).stdout
files = [f.strip() for f in out.splitlines() if f.strip()]
# 排除推送工具自身
files = [f for f in files if f not in ("api_push.py", "push_changes.py")]

missing = [f for f in files if not (ROOT / f).is_file()]
if missing:
    raise SystemExit(f"文件缺失: {missing}")

print(f"待推送 {len(files)} 个变更文件:", flush=True)
for f in files:
    print(f"  {f}", flush=True)

sys.argv = ["api_push.py"] + files
api_push.main()
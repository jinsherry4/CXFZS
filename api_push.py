# 通过 GitHub API 推送仓库（绕过被阻断的 github.com:443 git 通道）
# 流程: Contents API 初始化 README -> Git Data API(blob->tree->commit->ref) 推送其余文件
# 用法: python api_push.py [文件...]  无参数=全量推送；指定文件=增量推送
import base64
import json
import subprocess
import sys
import time
from pathlib import Path

import requests

REPO_API = "https://api.github.com/repos/jinsherry4/CXFZS"
REPO_DIR = Path(__file__).parent
MAX_RETRY = 5


def get_token():
    cred = subprocess.run(
        ["git", "credential", "fill"],
        input="protocol=https\nhost=github.com\n\n",
        capture_output=True, text=True, cwd=REPO_DIR,
    ).stdout
    for line in cred.splitlines():
        if line.startswith("password="):
            return line.split("=", 1)[1]
    raise RuntimeError("no token")


def api(method, url, headers, payload=None, timeout=300):
    for attempt in range(1, MAX_RETRY + 1):
        try:
            r = requests.request(method, url, headers=headers,
                                 data=json.dumps(payload) if payload else None,
                                 timeout=timeout)
            if r.status_code in (200, 201, 202):
                return r.json()
            if r.status_code in (403, 404, 409, 502) and attempt < MAX_RETRY:
                print(f"  {r.status_code} 退避重试 {attempt}", flush=True)
                time.sleep(3 * attempt)
                continue
            raise RuntimeError(f"{method} {url} -> {r.status_code}: {r.text[:300]}")
        except requests.RequestException as e:
            if attempt == MAX_RETRY:
                raise
            print(f"  网络重试 {attempt}: {e}", flush=True)
            time.sleep(3 * attempt)


def main():
    token = get_token()
    headers = {
        "Authorization": f"Bearer {token}",
        "Accept": "application/vnd.github+json",
        "X-GitHub-Api-Version": "2022-11-28",
        "Content-Type": "application/json",
        "User-Agent": "cxfzs-push",
    }

    # 0. 检查 main 分支是否已存在（已初始化）
    r = requests.get(f"{REPO_API}/git/ref/heads/main", headers=headers, timeout=60)
    if r.status_code == 404:
        print("main 不存在，先用 Contents API 初始化 README...", flush=True)
        readme = (REPO_DIR / "README.md").read_bytes()
        api("PUT", f"{REPO_API}/contents/README.md", headers,
            {"message": "init", "content": base64.b64encode(readme).decode(),
             "branch": "main"})
        print("  README 已推送，main 分支已创建", flush=True)
    else:
        r.raise_for_status()

    # 取 main 最新 commit 作为基
    ref = api("GET", f"{REPO_API}/git/ref/heads/main", headers)
    base_sha = ref["object"]["sha"]
    base_commit = api("GET", f"{REPO_API}/git/commits/{base_sha}", headers)
    print(f"基线 commit: {base_sha[:10]}", flush=True)

    # 文件清单（排除推送脚本本身；命令行指定文件=增量推送）
    if len(sys.argv) > 1:
        files = [a.replace("\\", "/").strip("/") for a in sys.argv[1:]]
        for f in files:
            if not (REPO_DIR / f).is_file():
                raise SystemExit(f"文件不存在: {f}")
        msg = "修复: blue_cube_5 穿模(出生点y=-3.75)+航点同步+scan_filter孤儿清理+RViz清场/动态XAUTH"
    else:
        out = subprocess.run(["git", "-c", "core.quotepath=false", "ls-files"],
                             capture_output=True, text=True, encoding="utf-8", cwd=REPO_DIR)
        files = [f for f in out.stdout.splitlines() if f.strip() and f != "api_push.py"]
        msg = "推送全部源码/面板/脚本/文档/演示视频（LLM解析+Nav2导航+避障+抓放+Web面板）"
    print(f"待推送文件: {len(files)} 个", flush=True)

    # 1. 逐文件建 blob
    tree_items = []
    for i, f in enumerate(files):
        data = (REPO_DIR / f).read_bytes()
        blob = api("POST", f"{REPO_API}/git/blobs", headers,
                   {"content": base64.b64encode(data).decode(), "encoding": "base64"})
        tree_items.append({"path": f, "mode": "100644", "type": "blob", "sha": blob["sha"]})
        if (i + 1) % 20 == 0 or i == len(files) - 1:
            print(f"  blob 进度 {i + 1}/{len(files)}", flush=True)

    # 2. 建 tree（基于现有 tree，覆盖同名文件）
    print("创建 tree...", flush=True)
    tree = api("POST", f"{REPO_API}/git/trees", headers,
               {"base_tree": base_commit["tree"]["sha"], "tree": tree_items})

    # 3. 建 commit
    print("创建 commit...", flush=True)
    commit = api("POST", f"{REPO_API}/git/commits", headers,
                 {"message": msg, "tree": tree["sha"], "parents": [base_sha]})

    # 4. 更新 main ref
    print("更新 main 分支...", flush=True)
    api("PATCH", f"{REPO_API}/git/refs/heads/main", headers,
        {"sha": commit["sha"], "force": False})

    print(f"完成! commit={commit['sha'][:10]}", flush=True)


if __name__ == "__main__":
    main()

# CXFZS — 大模型技术创新赛 机器人任务系统

全球校园人工智能算法精英大赛（大模型技术创新赛）参赛系统：**大模型指令解析 + Nav2 自主导航 + 动态避障 + 机械臂抓取放置 + Web 状态面板**，一镜到底完整任务流。

> 演示视频见 `demo/demo_full_task_flow_20260908.mp4`（14 分 39 秒，两轮任务零卡死一镜到底）

## 系统能力

| 模块 | 说明 |
|------|------|
| **LLM 指令解析** | `llm_parser` 调用 DeepSeek API，将自然语言题目（衣柜/宠物等数学题）解析为结构化指令（颜色→数量→目标区），带缓存与 MOCK 降级 |
| **任务调度** | `mission_node` 动态贪心调度，按当前位置重选下一目标，预算管理（290s）智能放弃边际任务并提前返程 |
| **自主导航** | Nav2 + 真值里程计定位垫片（`truth_odom` + `loc_shim`），迷宫地图全局/局部路径规划 |
| **动态避障** | `obstacle_mover` 双移动障碍巡逻（幅度 2.0m/1.4m，跨任务路线），实测直接避让 + 全局改道双策略 |
| **机械臂抓放** | `arm_controller` 预抓取吸附方案，实测抓取成功率 100%，落区精准（z=0.015m 无悬空） |
| **Web 面板** | `dashboard.html` 经 rosbridge 实时显示：题目 / 解析结果 / SLAM 地图与路径 / 相机画面 / 货物进度 / 工作状态 |

实测成绩（2026-09-08 连续录制验证 r20–r22）：两轮任务零卡死、零悬空，6 抓取 / 6 放置全部成功，详见 `docs/technical_proposal.docx`。

## 环境要求

- **Ubuntu 22.04 + ROS 2 Humble**（推荐直接使用大赛官方 U2204 虚拟机镜像，已预装 Gazebo / Nav2 / rosbridge）
- `python3-colcon-common-extensions`（官方镜像已含）
- DeepSeek API Key（[获取地址](https://platform.deepseek.com/)，无 Key 也可用 MOCK 模式跑通全流程）

## 快速部署（在 2204 虚拟机内）

```bash
# 1. 把本仓库克隆/拷贝到虚拟机（例如经宿主机共享目录或 scp）
#    以下假设仓库在 ~/CXFZS

# 2. 部署 ROS 工作区源码
mkdir -p ~/dev_ws/src
cp -r ~/CXFZS/ros_ws/src/yzbot ~/dev_ws/src/

# 3. 编译（首次约 2-3 分钟）
cd ~/dev_ws
colcon build --symlink-install
source install/setup.bash

# 4. 部署运行脚本与面板
cp ~/CXFZS/scripts/*.sh ~/CXFZS/scripts/*.py ~/
chmod +x ~/*.sh
mkdir -p ~/dashboard ~/comp_logs ~/comp_tools
cp ~/CXFZS/dashboard/* ~/dashboard/
cp ~/CXFZS/tools/comp_tools/* ~/comp_tools/
cp ~/CXFZS/tools/fastdds_udp.xml ~/

# 5. 配置 API Key
cp ~/CXFZS/competition_env.sh.example ~/competition_env.sh
nano ~/competition_env.sh   # 填入你的 DEEPSEEK_API_KEY

# 6. 一键启动
bash ~/start_all.sh
```

## 使用流程

```bash
# 启动（脚本自动完成：Gazebo → Nav2 → 定位垫片 → rosbridge → 任务链）
bash ~/start_all.sh

# 自检（12 项健康检查）
bash ~/comp_tools/boot_verify.sh

# 出题（触发 LLM 解析与任务执行）
ros2 run competition_bringup question_bridge

# 查看任务状态
ros2 topic echo /mission/work_state

# 停止全部
bash ~/comp_stop.sh
```

**Web 面板**（可选，可视化状态反馈）：

```bash
# 面板服务 + 浏览器（VM 桌面内执行）
bash ~/panel_final.sh
# 或无界面验证: 浏览器打开 http://<虚拟机IP>:8080/dashboard.html
# 面板经 rosbridge(ws://<虚拟机IP>:9090) 订阅话题
```

**MOCK 模式**（无 API Key 联调，使用预设题目）：

```bash
MISSION_MOCK=1 bash ~/start_all.sh
```

## 仓库结构

```
CXFZS/
├── ros_ws/src/yzbot/              # ROS 2 工作区源码
│   ├── competition_bringup/        #   核心包：调度/LLM解析/抓放/障碍/出题
│   ├── bot_navigation/             #   Nav2 导航配置（v2 地图, 无 AMCL 自愈型）
│   ├── mybot/ mybot_description/   #   机器人模型与仿真
│   └── IFRA_LinkAttacher/          #   抓取吸附插件
├── dashboard/                      # Web 状态面板（题目/地图/路径/相机/进度）
├── scripts/                        # 一键启动/面板/布局/定位垫片等脚本
├── tools/                          # 自检工具 + FastDDS 配置
├── docs/                           # 技术方案文档（1万字, 6创新点, 实测数据）
├── demo/                           # 完整任务流演示视频（14分39秒一镜到底）
└── competition_env.sh.example      # API Key 配置模板
```

## 核心节点（competition_bringup）

| 节点 | 职责 |
|------|------|
| `mission_node` | 任务调度：贪心选目标、预算管理、状态机 |
| `llm_parser` | LLM 指令解析（DeepSeek，JSON 结构化输出，缓存 + MOCK 降级） |
| `question_bridge` | 出题桥接：发布题目并联动任务链 |
| `arm_controller` | 机械臂预抓取 + 吸附控制 |
| `obstacle_mover` | 双移动障碍巡逻（跨任务路线） |
| `cmd_vel_watchdog` | 速度看门狗（卡死检测与自愈） |
| `carry_follower` | 搬运物随行 |
| `scan_filter` | 激光过滤 |

## 注意事项

1. **每轮任务前建议全新启动**：长时间运行（>20 分钟）会话老化可能引起激光漂移，`start_all.sh` 已内置全量进程清理与出生点校验。
2. **面板白屏**：VMware 环境下若浏览器白屏，脚本已自动设置 `WEBKIT_DISABLE_COMPOSITING_MODE=1`。
3. **API Key 安全**：切勿把真实 Key 提交进仓库，统一放 `~/competition_env.sh`（已 gitignore）。
4. **同题缓存**：相同题目文本命中缓存不重复调用 API，`~/comp_logs/` 有完整日志。

## 许可

内部备赛交流用途。`yzbot` 原始框架保留其原 LICENSE。

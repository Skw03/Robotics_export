# 《机器人技术》课程项目-阶段二文档

## 0. 基本信息

- 课程名称：《机器人技术》
- 学期：2025-2026 学年第 2 学期
- 项目类型：选项 A（工程应用项目）
- 项目题目：Office 场景下移动机器人仿真系统设计与中期验证
- 仓库链接：https://github.com/Skw03/Robotics_export  
- 小组成员：2356218 孙凯文、2351875 李璐巍、2354275 邱婉盈

---

## 1. 阶段二目标与交付范围

本阶段目标是给出完整系统方案，覆盖：

1. 基本功能定义
2. 逻辑架构与技术架构
3. 仿真环境搭建方案
4. 数据与模型资源说明
5. 可执行技术路线
6. 对课程技术要求（2.1）的逐条覆盖情况

本报告为阶段二可提交版本，已回填当前实验矩阵的结构化结果；个别运行截图仍可在最终提交前补充。

---

## 2. 项目方案总览

### 2.1 场景与目标

系统采用 Office 单一 world，完成两类核心任务：

- `delivery`（配送）
- `patrol`（巡检）

项目面向室内服务机器人应用，强调“可运行、可验证、可量化对比”。

### 2.2 功能清单

- 任务调度与执行（delivery / patrol）
- 路径规划与局部避障
- 定位与地图（AMCL + OccupancyGrid 验证链）
- 自然语言命令交互（LLM + fallback）
- 实验自动化与结果落盘（CSV/JSONL）

### 2.3 架构分层

- 交互层：自然语言命令输入与任务触发
- 任务层：任务模板、语义点、动作派发
- 导航层：规划器、控制器、局部避障策略
- 感知定位层：LaserScan、地图、TF、AMCL
- 仿真层：ROS 2 + Gazebo world

---

## 3. 技术实现与代码入口

### 3.1 任务与实验

- 任务派发：`robotics_scenario/scripts/course_task_dispatcher.py`
- 实验执行：`robotics_scenario/scripts/course_experiment_runner.py`
- profile 生成：`robotics_nav2/scripts/generate_nav2_profile.py`
- 一键实验：`tools/run_stage2_experiments_wsl.sh`

### 3.2 感知、地图、定位

- 合成激光：`office/scripts/office_synthetic_lidar.py`
- 栅格映射验证：`office/scripts/office_scan_mapper.py`
- Office 感知 launch：`office/launch/office_perception_mapping.launch.xml`
- AMCL 参数：`robotics_nav2/param/office_nav2.yaml` / `office_stage2_demo.yaml`

### 3.3 人机交互与 LLM

- 自然语言命令：`office/scripts/office_llm_command.py`
- 支持 OpenAI Responses API
- 支持 `--force-fallback` 兜底解析
- 支持 `--save-json` 保存解析/执行证据

---

## 4. 对照课程技术要求（2.1）逐条说明

### 4.1 感知与环境建模

- 已完成：
  - 使用 LaserScan 模态进行环境感知
  - 输出 OccupancyGrid（`/map`）验证链路
  - 已具备 AMCL 实时定位配置
- 验证入口：`office/launch/office_perception_mapping.launch.xml` 启动合成 LaserScan 与 OccupancyGrid 链路，核心节点为 `office_synthetic_lidar.py` 和 `office_scan_mapper.py`。
- 当前证据：`docs/office_stage2_update_summary.md` 已记录 `/scan`、`/map`、TF 链路的静态验证方式；本次未启动完整 WSL ROS 2 runtime 进行在线录屏，因此未追加在线截图和话题频率截图。
- 定位观测：AMCL 与 map_server 已能随 Office/Nav2 launch 启动；已有日志显示 lifecycle 节点进入 bringup/cleanup 流程，但缺少可复算的定位收敛时间与漂移量 CSV，因此本报告不填具体漂移数值。

### 4.2 运动控制与避障

- 已完成：
  - 基础运动控制通过 Nav2 控制栈实现
  - 已实现 2 种避障配置可切换
    - `collision_monitor`
    - `baseline_costmap`
- 对比口径：`collision_monitor` 与 `baseline_costmap` 两种避障配置均通过 `tools/run_stage2_experiments_wsl.sh` 进入实验矩阵，理论上分别覆盖 delivery 与 patrol 两类任务。
- 本次 `TRIALS=5` 实验已生成 4 组 CSV/JSONL。`baseline_costmap` 两个规划器在 delivery/patrol 上均达到 `100.00%` 成功率；`collision_monitor` 下 `smac_2d` 为 `100.00%`，`navfn_astar` 的 delivery 为 `60.00%`、patrol 为 `100.00%`。
- 失败案例：`navfn_astar + collision_monitor / delivery` 出现 `FAILED=1` 与 `TIMEOUT=1`；其余 7 个任务分组无失败样本。修复后不再以 action server 不可用作为主要阻断点。

### 4.3 路径规划

- 已完成：
  - 2 种规划算法可切换
    - `navfn_astar`
    - `smac_2d`
- 对比口径：规划器矩阵包含 `navfn_astar` 与 `smac_2d`，由 `robotics_nav2/scripts/generate_nav2_profile.py` 基于 `office_nav2.yaml` 生成运行参数。
- 统计结论：`smac_2d` 两种避障配置均达到 `100.00%` 成功率；`navfn_astar` 在 `baseline_costmap` 下为 `100.00%`，在 `collision_monitor` 下 delivery 成功率降至 `60.00%`。
- 参数选择依据：`navfn_astar` 作为 Nav2 经典全局规划基线，适合规则栅格地图的可解释对比；`smac_2d` 作为更面向网格搜索与代价约束的规划器，用于检验在 Office 狭窄通道和绕行场景下的鲁棒性。本次差异主要体现为 `collision_monitor` 下的稳定性与耗时波动。

### 4.4 应用场景

- 已完成：
  - 2 个独立任务逻辑（delivery/patrol）
  - 任务语义点与路线已定义
- 成功判据：实验脚本以每条记录的 `accepted=true` 且 `status/task_status` 为成功状态作为任务成功；`elapsed_sec` 记录任务耗时。
- 失败判据：`status=TIMEOUT` 计为超时失败，`status=ERROR` 计为运行异常，`accepted=false` 或任务被拒绝计为调度失败。
- 最终评估表口径：按 `task` 分为 delivery/patrol，按 `planner_profile + avoidance_profile` 分组统计样本数、成功数、成功率、平均耗时、超时率和失败类型分布；实际汇总见 5.4 与 `experiment_results/stage2_matrix/stage2_matrix_summary.csv`。

### 4.5 人机交互

- 已完成：
  - 可运行自然语言接口（dry-run / execute）
  - 解析结果可保存 JSON 证据
- 命令覆盖：已保留 `office_llm_command.py --force-fallback` 作为本地规则解析验证入口，覆盖中文配送、中文巡检、英文 delivery、英文 patrol 四类基本表达。
- 稳定性口径：对每条命令记录解析任务类型、是否 fallback、是否执行、错误信息和解析延迟；当前已有 dry-run 级静态验证，缺少批量 JSON 记录，因此不填解析正确率百分比。

### 4.6 LLM / VLM / VLA 集成

- 已完成：
  - 接入 LLM 进行任务语义解析
  - 输出解析延迟 `parse_latency_ms`
  - 支持失败回退
- 延迟统计口径：LLM 解析结果以 `parse_latency_ms` 记录，正式提交时按 JSON 证据计算平均值、P95 和最大值。
- 可靠性口径：以人工标注任务类型为基准，统计 LLM 输出正确率、误解析率、fallback 接管率和非法输出率。
- 已知失败类型与修复策略：网络超时或无 API Key 时使用 keyword fallback；模型输出非 delivery/patrol 时拒绝或降级；语义歧义命令进入人工复核或二次确认。

### 4.7 系统与部署分析

- 已完成：
  - 仿真软件架构与运行边界已明确
- 硬件平台建议：真实部署可选差速轮式底盘、2D LiDAR、轮速里程计、IMU 与板载工控机/NUC；课程阶段优先保证 ROS 2、Nav2、LaserScan、OccupancyGrid 与 AMCL 链路可迁移，暂不引入机械臂或复杂 VLA 执行端。

sim-to-real 主要风险与应对：

| 风险 | 应对策略 |
|---|---|
| LiDAR 噪声、玻璃反射、黑色物体误检 | 对 LaserScan 做滤波，调整 AMCL 激光模型参数 |
| 地图与真实环境不一致 | 真实场地重新建图，保留局部 costmap 在线更新 |
| 执行器延迟与轮胎打滑 | 降低速度/加速度，标定轮径和轴距，重调 controller 参数 |
| 初始定位偏差 | 固定起点，使用 `initialpose`，建立重定位检查流程 |
| 动态障碍 | 使用更保守的 inflation/collision monitor，加入暂停与人工接管 |
| LLM 网络失败 | 保留规则解析 fallback，设置超时和置信度阈值 |

---

## 5. 阶段二实验设计（最终采用方案）

### 5.1 设计原则

为降低变量耦合并适配当前仓库结构，实验采用：

- 单一 world：`office`
- 双任务：`delivery` + `patrol`
- 多算法对比：规划器 × 避障策略
- 多次重复：每组 `TRIALS=3~5`

### 5.2 实验矩阵

1. `navfn_astar + collision_monitor`（delivery + patrol）
2. `smac_2d + collision_monitor`（delivery + patrol）
3. `navfn_astar + baseline_costmap`（delivery + patrol）
4. `smac_2d + baseline_costmap`（delivery + patrol）

### 5.3 运行命令

```bash
WORKSPACE=$HOME/ros2_ws \
PROJECT_ROOT=/mnt/c/Users/QZB/Desktop/Robotics_export \
OUTPUT_DIR=/mnt/c/Users/QZB/Desktop/Robotics_export/experiment_results/stage2_matrix \
bash /mnt/c/Users/QZB/Desktop/Robotics_export/tools/run_stage2_experiments_wsl.sh
```

可选（自定义重复次数）：

```bash
WORKSPACE=$HOME/ros2_ws \
PROJECT_ROOT=/mnt/c/Users/QZB/Desktop/Robotics_export \
TRIALS=5 \
LAUNCH_WAIT_SEC=150 \
LAUNCH_TIMEOUT_SEC=1500 \
RESULT_TIMEOUT_SEC=120 \
OUTPUT_DIR=/mnt/c/Users/QZB/Desktop/Robotics_export/experiment_results/stage2_matrix \
bash /mnt/c/Users/QZB/Desktop/Robotics_export/tools/run_stage2_experiments_wsl.sh
```

### 5.4 结果文件路径

- 输出目录：`experiment_results/stage2_matrix`
- 日志目录：`experiment_results/stage2_matrix/logs`
- 结果格式：每组 `*.csv` + `*.jsonl`

本次正式实验产物（`TRIALS=5`，`RESULT_TIMEOUT_SEC=120`）：

| 类型 | 文件 | 状态 |
|---|---|---|
| launch 日志 | `experiment_results/stage2_matrix/logs/launch_navfn_astar_collision_monitor.log` | 已生成 |
| launch 日志 | `experiment_results/stage2_matrix/logs/launch_smac_2d_collision_monitor.log` | 已生成 |
| launch 日志 | `experiment_results/stage2_matrix/logs/launch_navfn_astar_baseline_costmap.log` | 已生成 |
| launch 日志 | `experiment_results/stage2_matrix/logs/launch_smac_2d_baseline_costmap.log` | 已生成 |
| CSV/JSONL | `experiment_results/stage2_matrix/office_delivery-patrol_navfn_astar_collision_monitor.csv` / `.jsonl` | 已生成 |
| CSV/JSONL | `experiment_results/stage2_matrix/office_delivery-patrol_smac_2d_collision_monitor.csv` / `.jsonl` | 已生成 |
| CSV/JSONL | `experiment_results/stage2_matrix/office_delivery-patrol_navfn_astar_baseline_costmap.csv` / `.jsonl` | 已生成 |
| CSV/JSONL | `experiment_results/stage2_matrix/office_delivery-patrol_smac_2d_baseline_costmap.csv` / `.jsonl` | 已生成 |
| 汇总表 | `experiment_results/stage2_matrix/stage2_matrix_summary.csv` / `.md` | 已生成 |

汇总统计表：

| planner | avoidance | task | trials | accepted | success | success_rate | mean_elapsed_sec | timeout_rate | failure_modes |
|---|---|---|---:|---:|---:|---:|---:|---:|---|
| navfn_astar | baseline_costmap | delivery | 5 | 5 | 5 | 100.00% | 7.656 | 0.00% | - |
| navfn_astar | baseline_costmap | patrol | 5 | 5 | 5 | 100.00% | 0.733 | 0.00% | - |
| navfn_astar | collision_monitor | delivery | 5 | 5 | 3 | 60.00% | 51.708 | 20.00% | FAILED=1; TIMEOUT=1 |
| navfn_astar | collision_monitor | patrol | 5 | 5 | 5 | 100.00% | 1.864 | 0.00% | - |
| smac_2d | baseline_costmap | delivery | 5 | 5 | 5 | 100.00% | 10.183 | 0.00% | - |
| smac_2d | baseline_costmap | patrol | 5 | 5 | 5 | 100.00% | 0.095 | 0.00% | - |
| smac_2d | collision_monitor | delivery | 5 | 5 | 5 | 100.00% | 8.252 | 0.00% | - |
| smac_2d | collision_monitor | patrol | 5 | 5 | 5 | 100.00% | 0.094 | 0.00% | - |

---

## 6. 中期结果与分析

### 6.1 结果汇总

- delivery 最优组合：`smac_2d + collision_monitor`、`smac_2d + baseline_costmap` 与 `navfn_astar + baseline_costmap` 均达到 `100.00%`；其中 `smac_2d + collision_monitor` 平均耗时最低（`8.252s`）。
- patrol 最优组合：4 个组合均达到 `100.00%`；其中 `smac_2d + collision_monitor` 平均耗时最低（`0.094s`），`smac_2d + baseline_costmap` 接近（`0.095s`）。
- 综合最优组合：优先选择 `smac_2d + collision_monitor`。该组合 delivery/patrol 均为 `100.00%`，且两个任务平均耗时均处于最优或并列最优区间。

### 6.2 关键指标

- 成功率：8 个任务分组中 7 个为 `100.00%`，仅 `navfn_astar + collision_monitor / delivery` 为 `60.00%`。
- 平均耗时：delivery 最低为 `smac_2d + collision_monitor = 8.252s`，patrol 最低为 `smac_2d + collision_monitor = 0.094s`；最高为 `navfn_astar + collision_monitor / delivery = 51.708s`。
- 超时率：仅 `navfn_astar + collision_monitor / delivery` 出现超时，超时率 `20.00%`；其余分组为 `0.00%`。
- 主要失败模式：`FAILED` 与 `TIMEOUT`，均集中在 `navfn_astar + collision_monitor / delivery`。

### 6.3 结论

阶段二已经补齐 Office-only 的任务入口、LLM/fallback 解析入口、感知地图验证入口和规划/避障实验矩阵脚本，并完成 `TRIALS=5` 的结构化结果落盘。修复启动时序、SMAC planner id 和阶段二短路线后，系统已能稳定生成 CSV/JSONL 与汇总表；综合结果显示 `smac_2d + collision_monitor` 是当前推荐组合，`navfn_astar + collision_monitor` 在 delivery 上仍存在超时/失败波动，后续应继续针对 collision monitor 与 NavFn 组合做控制参数和局部代价图调优。

---

## 7. 阶段三衔接计划

1. 完成实验矩阵全量运行与结果固化
2. 增加动态障碍与扰动工况，验证鲁棒性
3. 完成 sim-to-real 迁移策略细化与答辩页
4. 输出最终报告与演示材料（视频、图表、失败案例）

---

## 8. AI 工具使用专项说明（阶段二版）

### 8.1 工具与作用

- Coding Agent（Codex / ChatGPT）：用于代码阅读、脚本修改、实验流程整理与报告撰写支持。
- LLM API（OpenAI Responses API）：用于自然语言指令到任务类型（delivery/patrol）的语义解析。
- 本地规则解析器（keyword fallback）：在无 API Key、网络失败、接口超时或模型输出异常时作为兜底。
- 自动化实验脚本：用于批量执行规划/避障矩阵并输出结构化结果。

版本信息（阶段二提交口径）：

- ChatGPT / Codex：`GPT-5.x` 系列（用于开发协助与文档整理）
- OpenAI Responses API：`/v1/responses`（模型默认使用 `gpt-4o-mini`，可通过 `OPENAI_MODEL` 覆盖）
- ROS 2：Humble
- Gazebo：Gazebo Classic 11
- 导航栈：Nav2（Humble 对应版本）
- Python：3.10（Ubuntu 22.04 默认）

### 8.2 成本与资源

说明：阶段二成本以“估算值”披露，最终以实际账单与运行日志为准。

- API 调用成本（估算）：
  - 使用场景：自然语言指令解析、少量提示词调试
  - 估算调用次数：`30 ~ 120` 次
  - 单次 token 量级（输入+输出）：`300 ~ 1200 tokens`
  - 总 token 量级：约 `1e4 ~ 1e5`
  - 费用区间：按 `1e4 ~ 1e5` 总 token 量级估算约为低额美元成本；若使用本地 fallback 或 dry-run，则 API 成本可为 0。最终提交应以 OpenAI 控制台账单为准。
- 计算资源消耗（估算）：
  - CPU：仿真与导航主耗时在 ROS/Gazebo 运行阶段
  - 内存：中等负载（Office 单 world + Nav2 + 任务节点）
  - 典型单轮实验（4 组组合，TRIALS=3）耗时：约 `40 ~ 120` 分钟（受机器性能与场景状态影响）
- 人工投入：
  - 主要用于实验监控、失败样本筛选、结果复核与文档整理

### 8.3 局限性与失败案例

已观察到的局限性：

1. 语义歧义：自然语言输入过于宽泛时，LLM 可能将意图映射到错误任务类型。  
2. 网络依赖：API 调用受网络稳定性影响，可能出现超时或请求失败。  
3. 场景外指令：超出当前任务集合（仅 delivery/patrol）的命令会被强制映射，存在语义损失。  
4. 延迟波动：高峰时段响应延迟增大，影响交互实时性。

典型失败案例模板（阶段二记录格式）：

- 案例编号：AI-CASE-01
- 输入指令：`帮我处理一下办公室的事情`
- LLM 输出：可能在 delivery/patrol 之间不稳定
- 实际应有任务：语义不充分，需追问或人工确认
- 失败类型：歧义命令导致误解析风险
- fallback 是否接管：否，规则解析无法可靠判断
- 处置措施：限制输出 schema，仅允许 delivery/patrol；对低置信度或关键词缺失输入返回澄清提示。

### 8.4 验证与评估方法

为确保 AI 生成内容可用且可复现，采用以下验证流程：

1. 语义解析验证：
  - 对每条自然语言命令记录输入、解析结果、延迟、执行结果。
  - 对低置信度或 fallback 接管样本进行人工复核。
2. 任务执行验证：
  - 使用统一实验脚本重复运行，输出 CSV/JSONL。
  - 以成功率、耗时、超时率作为核心指标，避免主观结论。
3. 结果一致性验证：
  - 同一配置至少进行 `3` 次重复试验。
  - 若波动异常，增加重复次数并记录环境状态。
4. 文档可追溯性验证：
  - 报告中所有关键结论必须可回溯到对应结果文件与日志。
  - 对图表数据进行“源文件-统计表-报告文本”三方一致性检查。

提交前抽检建议：

- 语义命令样本抽检比例：不少于 `30%`
- 实验结果文件复算抽检比例：不少于 `20%`
- 关键结论（最优组合、失败模式）逐条人工复核：`100%`

---

## 9. 提交前检查清单

- [x] 小组成员与分工信息已填写
- [x] 所有 `TODO` 已处理为最终文本或明确说明缺少结构化结果
- [x] 日志附件路径已列出，CSV/JSONL 与汇总表已生成
- [x] 指标口径（成功率、耗时、超时）与脚本输出字段一致
- [x] AI 工具使用说明已补全

---

## 10. AI 工具使用情况披露

### 10.1 使用工具及作用

| 工具 | 作用 |
|---|---|
| ChatGPT / Codex | 辅助代码阅读、实验结果分析、报告结构整理、Markdown 文档撰写 |
| OpenAI Responses API | 作为机器人自然语言指令的 LLM 语义解析层 |
| 本地规则解析器 | 在无 API Key、网络失败或 LLM 超时时作为 fallback |
| 自动化实验脚本 | 批量运行规划/避障矩阵，生成 CSV、JSONL 和汇总结果 |
| `pptx` Skill | 辅助整理阶段二答辩 PPT 的结构、章节和幻灯片内容规划 |
| `ros2-engineering-skills` | 辅助 ROS 2 工程项目的架构梳理、Nav2/仿真流程检查和交付材料组织 |

版本口径：ChatGPT/Codex 使用 GPT-5 系列作为开发与文档协助；项目 LLM 入口使用 OpenAI Responses API，默认模型可由 `OPENAI_MODEL` 环境变量覆盖；ROS 2 目标环境为 Humble，Python 目标环境为 Ubuntu 22.04 默认 Python 3.10。

### 10.2 成本与资源消耗

成本口径：阶段二 API 调用主要集中在自然语言解析测试，估算调用 30~120 次、总 token 约 `1e4 ~ 1e5`；若使用 `--force-fallback`，API 成本为 0。实验运行时长按 4 组矩阵、每组 delivery/patrol、`TRIALS=3~5` 估算为 40~120 分钟，实际以 WSL 运行日志为准。

### 10.3 局限性与失败案例

失败样例：歧义命令可能在 delivery/patrol 间误解析；网络超时或 API Key 缺失时由 fallback 接管；超出任务集合的命令会被拒绝或要求澄清。影响范围仅限自然语言入口，不改变底层 ROS 2 任务脚本和规划/避障实验矩阵。

### 10.4 验证与评估方法

验证流程：AI 生成的代码、脚本和报告文本需经过人工审阅；语义命令样本抽检不少于 30%，实验结果文件复算抽检不少于 20%，最优组合、失败模式和报告关键数字必须 100% 回溯到 CSV/JSONL 或 launch 日志。

---

## 附录 A：建议附件目录（提交时配套）

- `experiment_results/stage2_matrix/*.csv`
- `experiment_results/stage2_matrix/*.jsonl`
- `experiment_results/stage2_matrix/stage2_matrix_summary.csv`
- `experiment_results/stage2_matrix/stage2_matrix_summary.md`
- `experiment_results/stage2_matrix/logs/*.log`
- 感知/定位运行截图（当前未生成，需在 WSL/ROS 2 runtime 中补录）
- 阶段二演示视频链接或文件索引

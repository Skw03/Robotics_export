# Commit Changes (2026-05-14)

## 1. 说明

本文档基于当前工作区 `changes`（`git status`）整理，覆盖本次待提交的全部改动文件，而非仅新增脚本。

## 2. 当前变更文件总览（changes 区）

### 2.1 Modified

- `robotics_scenario/scripts/_course_task_utils.py`
- `robotics_scenario/scripts/course_experiment_runner.py`

### 2.2 Added (Untracked)

- `STAGE2_SYSTEM_SOLUTION_2026.md`
- `tools/run_stage2_experiments.sh`
- `docs/COMMIT_CHANGES_2026-05-14.md`

## 3. 各文件改动内容

### 3.1 `robotics_scenario/scripts/_course_task_utils.py`

- 新增运行期评估指标采集逻辑：
  - 订阅 `/odom` 并累计路径长度 `path_length_m`；
  - 订阅 `/scan` 并记录最小障碍距离 `min_obstacle_dist_m`；
  - 基于阈值统计近碰事件数 `near_collision_events`。
- 在 `TaskDispatchNode.dispatch(...)` 中新增参数：
  - `collect_metrics`
  - `near_collision_threshold_m`
- 在任务返回结构 `task_spec` 中增加 `metrics` 字段，输出本轮采集结果。

### 3.2 `robotics_scenario/scripts/course_experiment_runner.py`

- 新增命令行参数：
  - `--near-collision-threshold`（默认 `0.25` m）。
- 在调用 `dispatch(...)` 时启用指标采集并透传阈值。
- 扩展结果记录字段（JSONL/CSV）：
  - `path_length_m`
  - `min_obstacle_dist_m`
  - `near_collision_events`
  - `near_collision_threshold_m`
- 异常分支下增加上述字段默认值，保证结果格式一致。

### 3.3 `tools/run_stage2_experiments.sh`

- 新增批量实验脚本，支持跨机器路径配置。
- 支持矩阵化运行：
  - 场景：`SCENES_CSV`
  - 任务：`TASKS_CSV`
  - 规划器：`PLANNERS_CSV`
  - 避障策略：`AVOIDANCE_CSV`
- 自动流程：
  - 生成参数组合 YAML；
  - 启动/停止 `indoor_delivery.launch.py`；
  - 调用 `course_experiment_runner.py` 执行多轮实验并输出结果。

### 3.4 `STAGE2_SYSTEM_SOLUTION_2026.md`

- 补充并正式化 `6.3`：
  - 明确未引入额外公开数据集；
  - 给出 LLM 调用成本估算口径（统计粒度、计算公式、结果呈现、异常处理）。
- 完成 `7.1`：
  - 明确传感器模态（2D LiDAR）与 `/scan` topic 用途；
  - 补充地图流程（离线建图、运行复用、局部在线更新）。
- 完成 `7.2`：
  - 明确主用定位方法为 AMCL；
  - 给出关键参数依据与定位稳定性证据采集计划。
- 更新第 11 节总览：
  - 移除已完成项（感知 topic、定位方法参数）；
  - 保留需实验数据支撑项。

### 3.5 `docs/COMMIT_CHANGES_2026-05-14.md`

- 新建提交改动说明文档（即本文件），用于记录当前 changes 区完整变更。

## 4. 本次改动带来的直接产出

- 评估数据字段从“仅成功率/耗时”扩展为“成功率/耗时/路径长度/近碰风险”。
- 新增可复现实验脚本，降低手工操作成本并统一实验流程。
- 系统方案文档补齐阶段二关键说明项，便于中期提交与答辩使用。

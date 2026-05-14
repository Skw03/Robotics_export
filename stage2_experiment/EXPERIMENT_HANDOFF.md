# 实验交接说明（给下一位同学）

## 1. 目标

本实验用于产出阶段二评估所需核心数据，覆盖：

- 导航成功率与耗时；
- 路径长度；
- 最小障碍距离与近碰事件（碰撞风险代理指标）；
- 不同规划器/避障策略组合的对比结果。

## 2. 运行前准备

在仓库根目录执行：

```bash
export PROJECT_ROOT="$(pwd)"
source /opt/ros/humble/setup.bash
source "$PROJECT_ROOT/install/local_setup.bash"
```

如需指定工作区（可选）：

```bash
export WORKSPACE="$HOME/ros2_ws"
```

## 3. 一键运行实验

默认运行矩阵：

- 场景：`warehouse,office`
- 任务：`delivery,patrol,demo`
- 规划器：`navfn_astar,smac_2d`
- 避障：`baseline_costmap,collision_monitor`
- 每组轮次：`10`

命令：

```bash
bash "$PROJECT_ROOT/stage2_experiment/run_stage2_experiments.sh"
```

常用缩小规模命令（用于快速检查）：

```bash
SCENES_CSV=warehouse TRIALS=3 bash "$PROJECT_ROOT/stage2_experiment/run_stage2_experiments.sh"
```

## 4. 输出结果位置

默认输出目录：

`$PROJECT_ROOT/experiment_results/stage2_matrix/`

其中：

- `profiles/`：每组实验的参数 YAML（可复现依据）
- `logs/`：launch 日志（排错依据）
- `nav/`：每组实验结果 `.csv` 与 `.jsonl`

## 5. 跑完后必须做的事

1. 检查结果完整性
- 确认 `nav/` 下每个组合都有 `csv + jsonl` 文件；
- 随机抽查 CSV 是否包含以下字段：
  - `elapsed_sec`
  - `path_length_m`
  - `min_obstacle_dist_m`
  - `near_collision_events`

2. 计算汇总指标（至少）
- 每组合成功率：`accepted/status/task_status` 统计；
- 平均耗时：`elapsed_sec`；
- 平均路径长度：`path_length_m`；
- 平均最小障碍距离：`min_obstacle_dist_m`；
- 近碰事件均值/总数：`near_collision_events`。

3. 形成提交材料
- 产出“组合对比总表”（建议 CSV）；
- 产出至少 2 张图（成功率柱状图、耗时或路径长度箱线图）；
- 在 `STAGE2_SYSTEM_SOLUTION_2026.md` 中回填实验结论（7.3、7.4、8.x、11 节相关 TODO）。

4. 异常处理记录
- 若某组合失败或超时，保留对应 `logs/*.launch.log`；
- 在结论中单列失败模式（如 TIMEOUT、ERROR）与可能原因。

## 6. 参数调整建议（可选）

- 机器性能不足：增大 `LAUNCH_WAIT_SEC`，减少 `TRIALS`；
- 导航长任务超时：增大 `RESULT_TIMEOUT_SEC`；
- 近碰判定更保守：减小 `NEAR_COLLISION_THRESHOLD`（如 `0.20`）。

示例：

```bash
TRIALS=5 RESULT_TIMEOUT_SEC=300 NEAR_COLLISION_THRESHOLD=0.20 \
bash "$PROJECT_ROOT/stage2_experiment/run_stage2_experiments.sh"
```

## 7. LLM 评测（补充 7.6/7.7）

批量执行语义解析评测：

```bash
export PROJECT_ROOT="$(pwd)"
export OPENAI_API_KEY=YOUR_KEY
bash "$PROJECT_ROOT/stage2_experiment/run_llm_eval.sh"
```

输出目录：

`$PROJECT_ROOT/experiment_results/llm/runs/`

## 8. 一键汇总结果

将导航结果与 LLM 结果汇总为提交可用文件：

```bash
export PROJECT_ROOT="$(pwd)"
python3 "$PROJECT_ROOT/stage2_experiment/summarize_stage2_results.py"
```

输出目录：

`$PROJECT_ROOT/experiment_results/summary/`

- `nav_summary.csv`
- `llm_summary.json`

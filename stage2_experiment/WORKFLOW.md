# 阶段二实验执行流程（跑脚本 -> 拿结果 -> 写文档）

## 1. 环境准备

在仓库根目录执行：

```bash
export PROJECT_ROOT="$(pwd)"
source /opt/ros/humble/setup.bash
source "$PROJECT_ROOT/install/local_setup.bash"
```

## 2. 运行导航矩阵实验

用途：产出规划/避障对比、成功率、耗时、路径长度、近碰指标。

```bash
bash "$PROJECT_ROOT/stage2_experiment/run_stage2_experiments.sh"
```

## 3. 运行 LLM 评测

用途：产出解析成功率、时延、失败类型。

```bash
export OPENAI_API_KEY=YOUR_KEY
bash "$PROJECT_ROOT/stage2_experiment/run_llm_eval.sh"
```

## 4. 汇总结果

用途：生成提交用汇总文件。

```bash
python3 "$PROJECT_ROOT/stage2_experiment/summarize_stage2_results.py"
```

## 5. 确认最终输出

检查以下两个文件是否生成：

- `$PROJECT_ROOT/experiment_results/summary/nav_summary.csv`
- `$PROJECT_ROOT/experiment_results/summary/llm_summary.json`

## 6. 回填文档 TODO（`STAGE2_SYSTEM_SOLUTION_2026.md`）

基于结果回填：

- `7.3`：避障策略对比（成功率、耗时、近碰指标）
- `7.4`：规划算法对比（成功率、耗时、路径长度）
- `7.7`：LLM 评测（成功率、时延、失败类型）
- `8.x`：实验汇总结果与结论
- `11`：更新总览，移除已完成 TODO

## 7. 仍需人工补写的内容

- `7.6`：歧义指令样本与误判分析（从 LLM 评测样本中选例）
- `7.8`：硬件选型与 sim-to-real 论证
- 说明口径：当前“碰撞”采用近碰代理指标 `near_collision_events`

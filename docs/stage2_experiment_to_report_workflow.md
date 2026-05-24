# 阶段二流程指南：从实验运行到文档回填

## 1. 目标

本流程用于完成阶段二最后收尾，覆盖：

1. 运行实验矩阵
2. 收集与核对结果文件
3. 统计关键指标
4. 回填阶段二报告
5. 提交前一致性检查

---

## 2. 前置条件

在 WSL Ubuntu 22.04 环境中确认：

1. 已完成 workspace 构建并 `source`
2. 可运行 `ros2` 命令
3. 已存在脚本：
   - `tools/run_stage2_experiments_wsl.sh`
   - `robotics_scenario/scripts/course_experiment_runner.py`
4. 报告文件已存在：
   - `docs/stage2_midterm_report.md`

---

## 3. 运行实验

### 3.1 默认运行（推荐先跑一轮）

```bash
bash /Users/qiuqiu/Desktop/github/Robotics_export/tools/run_stage2_experiments_wsl.sh
```

### 3.2 正式运行（建议 TRIALS=5）

```bash
TRIALS=5 OUTPUT_DIR=/Users/qiuqiu/Desktop/github/Robotics_export/experiment_results/stage2_matrix \
bash /Users/qiuqiu/Desktop/github/Robotics_export/tools/run_stage2_experiments_wsl.sh
```

说明：

- world 固定：`office`
- task 固定：`delivery` + `patrol`
- 组合矩阵：
  1. `navfn_astar + collision_monitor`
  2. `smac_2d + collision_monitor`
  3. `navfn_astar + baseline_costmap`
  4. `smac_2d + baseline_costmap`

---

## 4. 获取结果文件

实验完成后检查目录：

- 结果目录：`/Users/qiuqiu/Desktop/github/Robotics_export/experiment_results/stage2_matrix`
- 日志目录：`/Users/qiuqiu/Desktop/github/Robotics_export/experiment_results/stage2_matrix/logs`

应至少看到以下 4 组结果（每组 `csv + jsonl`）：

1. `office_delivery-patrol_navfn_astar_collision_monitor.*`
2. `office_delivery-patrol_smac_2d_collision_monitor.*`
3. `office_delivery-patrol_navfn_astar_baseline_costmap.*`
4. `office_delivery-patrol_smac_2d_baseline_costmap.*`

快速检查：

```bash
ls -lh /Users/qiuqiu/Desktop/github/Robotics_export/experiment_results/stage2_matrix
ls -lh /Users/qiuqiu/Desktop/github/Robotics_export/experiment_results/stage2_matrix/logs
```

---

## 5. 统计关键指标（建议）

从 CSV 统计以下指标（按 task 分 delivery/patrol）：

1. 成功率（`accepted=true` 且状态成功）
2. 平均耗时（`elapsed_sec` 均值）
3. 超时率（`status=TIMEOUT` 占比）
4. 失败模式分布（`ERROR/TIMEOUT/REJECTED`）

建议先手工做一版汇总表（Excel/Markdown 均可），后续再画图。

---

## 6. 回填阶段二文档

目标文件：

- `docs/stage2_midterm_report.md`

按下面顺序回填：

1. **5.4** 回填结果文件清单与汇总表来源  
2. **4.2 / 4.3** 回填避障与规划对比结论  
3. **4.4** 回填两个任务的成功/失败判据与指标  
4. **6.1 / 6.2 / 6.3** 回填最优组合、关键指标、总体结论  
5. **4.1 / 4.5 / 4.6** 回填截图、延迟统计、失败案例  
6. **8 / 10** 回填 AI 调用成本最终数字与验证流程细节  
7. **0** 回填成员和分工

---

## 7. 提交前检查清单（强制）

1. 报告中的数值能在 CSV/JSONL 中找到依据
2. 最优组合结论与汇总表一致
3. 所有 `TODO-*` 都已处理（删除或替换为最终文本）
4. 附件路径真实可访问
5. 图表标题、单位、口径一致（成功率/耗时/超时率）

可用命令检查是否仍有 TODO：

```bash
rg -n "TODO-" /Users/qiuqiu/Desktop/github/Robotics_export/docs/stage2_midterm_report.md
```

---

## 8. 常见问题排查

1. 没有生成 CSV/JSONL  
- 先看 `logs/launch_*.log` 是否 launch 失败  
- 检查 ROS 环境是否正确 source

2. 结果里大量 TIMEOUT  
- 适当调大 `RESULT_TIMEOUT_SEC`  
- 增加 `LAUNCH_WAIT_SEC` 确保系统稳定后再发任务

3. 某些组合完全失败  
- 先保留失败数据，不要删  
- 在报告中作为失败模式进行分析，这是有效结果

---

## 9. 最终交付物建议

1. `docs/stage2_midterm_report.md`（最终版）
2. `experiment_results/stage2_matrix/*.csv`
3. `experiment_results/stage2_matrix/*.jsonl`
4. `experiment_results/stage2_matrix/logs/*.log`
5. 关键运行截图与（可选）演示视频索引


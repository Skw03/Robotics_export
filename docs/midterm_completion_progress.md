# 中期完成进度记录

更新时间：2026-05-06

## 1. 本轮已完成内容

本项目继续按选项 A 工程应用项目推进，主题为室内物流与巡检移动机器人仿真系统。当前主线保持两个仿真场景：`warehouse` 与 `office`，每个场景支持 `delivery` 与 `patrol` 两类任务。

### 代码与功能

| 模块 | 文件 | 完成内容 |
| --- | --- | --- |
| LLM 语义任务入口 | `robotics_scenario/scripts/course_llm_command.py` | 使用 OpenAI Responses API 将自然语言解析为 `scene/task/confidence/rationale`，并保留本地关键词 fallback |
| 中文自然语言 fallback | `robotics_scenario/scripts/course_nl_command.py` | 修复中文关键词识别，支持办公室/仓库、配送/巡检等表达 |
| 实验记录脚本 | `robotics_scenario/scripts/course_experiment_runner.py` | 运行 preset task 并输出 CSV/JSONL，记录 `accepted/status/task_status/elapsed_sec/route/error` |
| Nav2 实验 profile | `robotics_nav2/scripts/generate_nav2_profile.py` | 支持生成 NavFn A*、Smac2D、baseline costmap、collision monitor 对比配置 |
| 启动参数 | `robotics_nav2/launch/indoor_delivery.launch.py` | 增加 `nav2_params_file` 参数，便于传入实验配置 |
| 构建安装 | `robotics_scenario/CMakeLists.txt`、`robotics_nav2/CMakeLists.txt` | 安装新增脚本，保证 `ros2 run` 可调用 |

### 文档与交付物

| 类型 | 文件 | 用途 |
| --- | --- | --- |
| 系统设计说明 | `docs/option_a_system_design.md` | 中期答辩系统方案与作业要求映射 |
| 实验协议 | `docs/evaluation_protocol.md` | 规划/避障/LLM 可靠性实验流程 |
| AI 使用说明 | `docs/ai_tools_usage.md` | 满足最终提交 AI 工具专项说明要求 |
| 仿真到真机分析 | `docs/sim_to_real_analysis.md` | 硬件选型与迁移障碍分析 |
| 中期 PPT 提纲 | `docs/midterm_presentation_outline.md` | 阶段二答辩讲述线 |
| 中期 PPT | `deliverables/robotics_option_a_midterm_deck.pptx` | 可编辑 PowerPoint 初稿 |
| 最终报告草稿 | `deliverables/robotics_option_a_final_report_draft.docx` | 可编辑 Word 报告草稿 |
| 实验工作簿 | `deliverables/robotics_experiment_template.xlsx` | 实验数据与 LLM 测试模板 |

## 2. 已验证项

| 检查项 | 状态 | 说明 |
| --- | --- | --- |
| Python 语法检查 | 已通过 | 新增脚本和 launch 文件均通过 `py_compile` |
| LLM fallback dry-run | 已通过 | 无 API key 时可解析 `办公室巡检一圈` 为 `office/patrol` |
| Nav2 profile 生成器 dry-run | 已通过 | 可生成实验 YAML 文件 |
| DOCX/PPTX/XLSX 文件结构 | 已通过 | Office 文件可作为 zip/xlsx 正常读取 |

## 3. WSL 构建结果

已执行，WSL 发行版：`Ubuntu-22.04`。

构建命令按 README 执行：

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
mkdir -p ~/ros2_ws/src
ln -sfn /mnt/e/Robotic/course_robot_ws/src/Robotics_export ~/ros2_ws/src/Robotics
colcon build --symlink-install --packages-select \
  robotics_description \
  robotics_gazebo \
  robotics_interfaces \
  robotics_localization \
  robotics_nav2 \
  robotics_scenario
source ~/ros2_ws/install/local_setup.bash
```

结果：构建成功，6 个包完成，耗时约 2 分 21 秒。

构建日志：

- `experiment_results/real_midterm/logs/colcon_build_20260506_011113.log`

构建备注：

- `robotics_scenario` 有 CMake warning：检测到 `behaviortree_cpp_v3` 版本 `3.8.7`，但未导致构建失败。
- colcon 提示工作区中有 Nav2 相关源码包但本次未构建，实际使用 `/opt/ros/humble` 中的 `nav2_common/nav2_msgs/nav2_util/nav2_behavior_tree`。

## 4. 中期真实仿真实验结果

已执行 4 条真实任务，每条 `trials=1`，配置为 `planner_profile=configured`、`avoidance_profile=collision_monitor`：

| Scene | Task | Planner profile | Avoidance profile | Status | Task status | Elapsed sec | Error |
| --- | --- | --- | --- | --- | --- | --- | --- |
| warehouse | delivery | configured | collision_monitor | FAILED | FAILED | 50.887 | Action accepted, Nav2 goal failed |
| warehouse | patrol | configured | collision_monitor | FAILED | FAILED | 124.367 | Action accepted, Nav2 goal failed |
| office | delivery | configured | collision_monitor | FAILED | FAILED | 120.56 | Action accepted, Nav2 goal failed |
| office | patrol | configured | collision_monitor | FAILED | FAILED | 589.341 | Action accepted, Nav2 goal failed |

真实数据文件：

- `experiment_results/real_midterm/warehouse_delivery_configured_collision_monitor.csv`
- `experiment_results/real_midterm/warehouse_patrol_configured_collision_monitor.csv`
- `experiment_results/real_midterm/office_delivery_configured_collision_monitor.csv`
- `experiment_results/real_midterm/office_patrol_configured_collision_monitor.csv`

场景日志：

- `experiment_results/real_midterm/logs/warehouse_launch_20260506_011336.log`
- `experiment_results/real_midterm/logs/office_launch_20260506_011842.log`

关键失败线索：

- warehouse 日志出现 `The goal sent to the planner is off the global costmap`，失败目标包括 `(-1.59, 7.72)`，说明部分语义点与当前仓库地图边界/原点/分辨率可能不一致。
- office 日志出现 `RegulatedPurePursuitController detected collision ahead` 和 `Resulting plan has 0 poses in it`，说明局部控制/碰撞监控或某些路线段仍需调参。
- 四条任务均被 action server 接受，失败发生在 Nav2 执行阶段；这对中期汇报很有价值，可作为“已跑通任务调度闭环，但导航参数与语义点仍需校准”的真实发现。

## 5. 工作簿更新结果

已执行。`tools/fill_experiment_workbook.py` 已读取 `experiment_results/real_midterm/*.csv`，并写入：

- `deliverables/robotics_experiment_template.xlsx`

验证结果：

- `Experiment Template` sheet 已写入 4 条真实 WSL 运行记录。
- `Summary` sheet 公式仍存在，包括总 trial 数、完成 trial 数、成功率、平均完成耗时、错误 trial 数。
- `LLM Test Cases` sheet 保留测试模板，等待后续 API key 可用时填入真实 LLM 测试数据。

## 6. 后续待做

- 用本文件中的真实运行数据替换中期 PPT 和报告中的占位描述。
- 修正仓库语义点/地图坐标问题，优先检查 `charging_dock` 等点是否落在全局 costmap 内。
- 调整办公室局部控制和 collision monitor 参数，重点排查 `detected collision ahead` 和 `0 poses` 的路线段。
- 期末前扩展为 NavFn A* vs Smac2D、baseline costmap vs collision monitor 的多次重复实验。
- 若 OpenAI API key 可用，补 10 条中英文自然语言命令的 LLM 延迟、置信度和失败案例。
- 增加 RViz/Gazebo 截图，用于中期答辩的可视化证据。

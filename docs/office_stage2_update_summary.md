# Office Stage 2 Update Summary

## 更新目标

本次更新按 `office_stage2_design.md` 的 Office-only 阶段二设计补齐实现，重点是让 Office 分支具备可展示、可验证、可记录的阶段二入口：配送、巡检、清扫、自然语言任务解析、机器人状态查询、仿真 `/scan` 与 `/map` 感知地图验证。

## 新增功能

### 1. Office 感知与地图验证

新增文件：

- `office/launch/office_perception_mapping.launch.xml`
- `office/scripts/office_synthetic_lidar.py`
- `office/scripts/office_scan_mapper.py`

启动入口：

```bash
ros2 launch office office_perception_mapping.launch.xml headless:=true use_rviz:=false
```

功能：

- `office_synthetic_lidar.py` 订阅 RMF `robot_state`，读取 `tinyRobot1` 位姿。
- 发布 `/scan`，类型为 `sensor_msgs/msg/LaserScan`。
- 发布 TF：`map -> odom -> base_footprint -> lidar_link`。
- `office_scan_mapper.py` 订阅 `/scan` 和 `robot_state`，将扫描点转换到 Office 地图坐标系。
- 发布 `/map`，类型为 `nav_msgs/msg/OccupancyGrid`。

该实现是稳定可复现的仿真 LaserScan / OccupancyGrid 验证链路，不依赖 Gazebo 原生 GPU LiDAR 插件。

### 2. Office-only 自然语言 / LLM 命令入口

新增文件：

- `office/scripts/office_llm_command.py`

入口：

```bash
ros2 run office office_llm_command.py "请巡检办公室所有检查点"
ros2 run office office_llm_command.py "把文件送到硬件办公室"
```

功能：

- 只面向 Office 场景。
- 只允许解析为 `delivery` 或 `patrol`。
- 默认 dry-run，只输出 JSON，不派发任务。
- 支持 `--execute` 调用 Office 任务入口。
- 支持 `--force-fallback` 本地关键词解析。
- 支持 OpenAI Responses API。
- 支持 `--save-json` 保存解析和执行证据。

示例：

```bash
ros2 run office office_llm_command.py --force-fallback "start an office patrol"
ros2 run office office_llm_command.py --execute "请巡检办公室所有检查点"
ros2 run office office_llm_command.py --save-json experiment_results/office_dynamic/llm/llm_patrol.json "please inspect the office"
```

### 3. Office 清扫任务入口

新增文件：

- `office/scripts/dispatch_clean`
- `office/scripts/gui_clean_test`
- `tools/run_office_gui_clean_wsl.sh`

入口：

```bash
ros2 run office dispatch_clean --use_sim_time
```

默认参数：

- 清扫点：`lounge`
- fleet：`tinyRobot`
- 机器人：`tinyRobot1`
- 垃圾房：`trash_room`
- 返回充电点：`tinyRobot1_charger`

WSL GUI 演示入口：

```bash
/mnt/e/Robotic/course_robot_ws/src/Robotics_export/tools/run_office_gui_clean_wsl.sh
```

### 4. Office 机器人状态查询

新增文件：

- `office/scripts/office_status`

入口：

```bash
ros2 run office office_status
```

功能：读取 fleet manager `/open-rmf/office_demos_fm/status/`，输出 robot name、map name、x/y/yaw、电量、最后完成请求、目标到达状态和 replan 标记。

示例：

```bash
ros2 run office office_status
ros2 run office office_status --robot tinyRobot1
ros2 run office office_status --watch 1.0
ros2 run office office_status --json
```

## 修改的现有文件

### `office/CMakeLists.txt`

新增安装入口：

- `scripts/dispatch_clean`
- `scripts/gui_clean_test`
- `scripts/office_status`
- `scripts/office_llm_command.py`
- `scripts/office_synthetic_lidar.py`
- `scripts/office_scan_mapper.py`

### `office/package.xml`

- 版本从 `0.1.0` 更新为 `0.2.0`。
- 描述更新为 Office-only stage-2 launch/task/LLM/status/perception/mapping wrappers。
- 新增依赖：`office_maps`、`office_fleet_adapter`、`rclpy`、`sensor_msgs`、`nav_msgs`、`geometry_msgs`、`tf2_ros`、`rmf_fleet_msgs`。

### `office/scripts/gui_delivery_test` 和 `office/scripts/gui_patrol_test`

移除硬编码 `/mnt/e/.../Robotics_export` 路径，改为通过 `PROJECT_ROOT` 或脚本相对路径定位项目根目录。

### `office_fleet_adapter/office_fleet_adapter/RobotClientAPI.py`

修复：

```python
requests.get(url, self.timeout)
```

改为：

```python
requests.get(url, timeout=self.timeout)
```

并统一 status URL 为 `/open-rmf/office_demos_fm/status/`。

### `tools/run_office_gui_patrol_wsl.sh`

补充 Office / RMF 运行依赖检查，使 patrol GUI 脚本和 delivery GUI 脚本行为一致。

### Python BOM 修复

移除 `office_tasks` 和 `office_fleet_adapter` 多个 Python 文件开头的 UTF-8 BOM，避免 Python 运行时报：

```text
SyntaxError: invalid non-printable character U+FEFF
```

## 当前验证结果

已完成静态验证：

```text
41 个 Python 文件 AST 解析通过
5 个 XML 文件解析通过
git diff --check 通过
```

已完成本地 dry-run 验证：

```bash
python office/scripts/office_llm_command.py --force-fallback "请巡检办公室所有检查点"
```

结果解析为 `patrol`。

```bash
python office/scripts/office_llm_command.py --force-fallback "把文件送到硬件办公室"
```

结果解析为 `delivery`。

已验证：

```bash
python office/scripts/office_status --help
```

能够正常输出命令帮助。

## 后续建议

在 WSL / Ubuntu ROS 2 Humble 环境中继续做运行级验证：

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/local_setup.bash
colcon build --symlink-install --packages-select \
  office office_gz office_demos office_maps office_assets \
  office_fleet_adapter office_tasks
source install/local_setup.bash
```

验证 Office 基础启动：

```bash
ros2 launch office office.launch.xml headless:=false
```

验证配送、巡检、清扫：

```bash
ros2 run office dispatch_delivery --use_sim_time
ros2 run office dispatch_patrol --use_sim_time
ros2 run office dispatch_clean --use_sim_time
```

验证状态：

```bash
ros2 run office office_status
```

验证感知地图：

```bash
ros2 launch office office_perception_mapping.launch.xml headless:=true use_rviz:=false
ros2 topic hz /scan
ros2 topic echo /scan sensor_msgs/msg/LaserScan --once --qos-reliability best_effort
ros2 topic echo /map nav_msgs/msg/OccupancyGrid --once
ros2 run tf2_ros tf2_echo base_footprint lidar_link
```

验证 LLM fallback 和执行：

```bash
ros2 run office office_llm_command.py --force-fallback "start an office patrol"
ros2 run office office_llm_command.py --force-fallback "deliver the file to the hardware office"
ros2 run office office_llm_command.py --execute --force-fallback "start an office patrol"
```

## 注意事项

- `tools/patch_wsl_robotics_gazebo.py` 是已有未跟踪文件，本次未修改。
- 当前验证主要是静态检查和不依赖 ROS runtime 的 dry-run。真实 `/scan`、`/map`、RMF 任务执行需要在 WSL / Ubuntu ROS 2 Humble 环境中启动 Office runtime 后验证。

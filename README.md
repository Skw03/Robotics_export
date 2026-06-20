# Robotics

Robotics 是一个基于 ROS 2 Humble 的课程工作空间，提供两个可运行的场景配置：

- `warehouse`：基于 AWS RoboMaker 小型仓库改编的标准仓库场景
- `office`：基于 RMF 办公室场景改编，适配本地 Gazebo Classic + Nav2 技术栈

## 系统要求

- Ubuntu 22.04
- ROS 2 Humble (`ros-humble-ros-base`)
- Gazebo 11 (`libgazebo-dev`)

## 1. 安装依赖

### 1.1 ROS 2 核心与 Gazebo

```bash
sudo apt-get update && sudo apt-get install -y \
  ros-humble-ros-base \
  libgazebo-dev \
  ros-humble-gazebo-ros-pkgs
```

### 1.2 导航栈 (Nav2)

```bash
sudo apt-get install -y \
  ros-humble-nav2-bringup \
  ros-humble-nav2-amcl \
  ros-humble-nav2-controller \
  ros-humble-nav2-navfn-planner \
  ros-humble-nav2-smac-planner \
  ros-humble-nav2-regulated-pure-pursuit-controller \
  ros-humble-nav2-collision-monitor \
  ros-humble-nav2-waypoint-follower \
  ros-humble-nav2-behaviors \
  ros-humble-nav2-map-server \
  ros-humble-nav2-lifecycle-manager \
  ros-humble-nav2-msgs \
  ros-humble-nav2-common \
  ros-humble-nav2-util
```

### 1.3 机器人描述与控制

```bash
sudo apt-get install -y \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-xacro \
  ros-humble-controller-manager \
  ros-humble-diff-drive-controller \
  ros-humble-joint-state-broadcaster \
  ros-humble-joint-trajectory-controller \
  ros-humble-interactive-marker-twist-server \
  ros-humble-teleop-twist-joy \
  ros-humble-joy \
  ros-humble-twist-mux
```

### 1.4 定位与感知

```bash
sudo apt-get install -y \
  ros-humble-robot-localization \
  ros-humble-imu-filter-madgwick \
  ros-humble-pcl-ros \
  ros-humble-pcl-conversions \
  ros-humble-spatio-temporal-voxel-layer \
  ros-humble-rclcpp-components
```

### 1.5 RMF (Robot Middleware Framework) — Office 场景必需

Office 场景依赖 RMF 进行车队管理、任务调度和仿真插件。

```bash
sudo apt-get install -y \
  ros-humble-rmf-dev \
  ros-humble-rmf-fleet-adapter-python \
  ros-humble-rmf-fleet-msgs \
  ros-humble-rmf-task-msgs \
  ros-humble-rmf-task-ros2 \
  ros-humble-rmf-traffic-ros2 \
  ros-humble-rmf-visualization \
  ros-humble-rmf-building-map-tools \
  ros-humble-rmf-building-sim-gz-classic-plugins \
  ros-humble-rmf-robot-sim-gz-classic-plugins \
  ros-humble-rmf-dispenser-msgs \
  ros-humble-rmf-lift-msgs \
  ros-humble-rmf-door-msgs \
  ros-humble-rmf-building-map-msgs \
  ros-humble-rmf-traffic-msgs \
  ros-humble-rmf-visualization-msgs \
  ros-humble-rmf-charger-msgs \
  ros-humble-rmf-site-map-msgs \
  ros-humble-rmf-websocket \
  ros-humble-ros-gz-bridge \
  ros-humble-ros-gz-sim \
  ros-humble-ros2launch \
  ros-humble-launch-xml \
  ros-humble-rviz2
```

### 1.6 Fleet Adapter Python 依赖

```bash
sudo apt-get install -y \
  python3-fastapi \
  python3-uvicorn \
  python3-pydantic \
  python3-requests \
  python3-flask-socketio \
  python3-pyproj \
  python3-numpy \
  python3-yaml
```

### 1.7 工具

```bash
sudo apt-get install -y \
  tmux \
  tmuxp \
  python3-pip
```

### 1.8 验证依赖安装

运行以下命令检查关键依赖是否安装成功：

```bash
# 检查 ROS 2
ros2 --version 2>/dev/null || echo "错误: ROS 2 Humble 未安装或未 source 环境"

# 检查 Nav2
dpkg -l ros-humble-nav2-bringup 2>/dev/null | grep -q '^ii' || echo "错误: ros-humble-nav2-bringup 未安装"

# 检查 RMF
dpkg -l ros-humble-rmf-fleet-adapter-python 2>/dev/null | grep -q '^ii' || echo "错误: ros-humble-rmf-fleet-adapter-python 未安装"
dpkg -l ros-humble-rmf-building-sim-gz-classic-plugins 2>/dev/null | grep -q '^ii' || echo "错误: ros-humble-rmf-building-sim-gz-classic-plugins 未安装（Gazebo 中机器人无法移动）"
dpkg -l ros-humble-rmf-robot-sim-gz-classic-plugins 2>/dev/null | grep -q '^ii' || echo "错误: ros-humble-rmf-robot-sim-gz-classic-plugins 未安装"

# 检查 Gazebo
gazebo --version 2>/dev/null || echo "错误: Gazebo 未安装"

# 检查 Python 依赖
python3 -c "import fastapi; import uvicorn; import pydantic; import requests; import yaml; import numpy" 2>/dev/null || echo "错误: 缺少 Python 依赖 (fastapi/uvicorn/pydantic/requests/yaml/numpy)"

# 检查 Nav2 共享库
ldd /opt/ros/humble/lib/libcontroller_server_core.so 2>/dev/null | grep "not found" && echo "错误: Nav2 controller_server 缺少共享库（已知问题，见故障排除 6.4）"
```

## 2. 构建

### 2.1 原生 Ubuntu 构建

```bash
cd ~/ros_ws
source /opt/ros/humble/setup.bash

# 构建所有包
colcon build --symlink-install

# 或仅构建 Office 相关包
colcon build --symlink-install --packages-select \
  office office_gz office_demos office_maps office_assets \
  office_fleet_adapter office_tasks \
  robotics_description robotics_gazebo robotics_interfaces \
  robotics_localization robotics_nav2 robotics_scenario

source install/setup.bash
```

### 2.2 WSL 快速启动

```bash
wsl -d Ubuntu-22.04
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
source install/setup.bash
```

### 2.3 验证构建

构建完成后，验证所有必需的包和文件是否就绪：

```bash
# 检查关键包是否可找到
ros2 pkg prefix office 2>/dev/null || echo "错误: 找不到 'office' 包 — 是否已 source install/setup.bash？"
ros2 pkg prefix office_maps 2>/dev/null || echo "错误: 找不到 'office_maps' 包"
ros2 pkg prefix office_fleet_adapter 2>/dev/null || echo "错误: 找不到 'office_fleet_adapter' 包"
ros2 pkg prefix robotics_nav2 2>/dev/null || echo "错误: 找不到 'robotics_nav2' 包"
ros2 pkg prefix robotics_scenario 2>/dev/null || echo "错误: 找不到 'robotics_scenario' 包"

# 检查地图文件
MAP_DIR=$(ros2 pkg prefix office_maps 2>/dev/null)/share/office_maps
[ -f "$MAP_DIR/maps/office/office.building.yaml" ] || echo "错误: Office 建筑地图文件不存在: $MAP_DIR/maps/office/office.building.yaml"
[ -d "$MAP_DIR/generated_maps/office" ] || echo "错误: Office 生成地图目录不存在: $MAP_DIR/generated_maps/office"

# 检查 Nav2 地图文件
NAV2_DIR=$(ros2 pkg prefix robotics_nav2 2>/dev/null)/share/robotics_nav2
[ -f "$NAV2_DIR/map/office_map.yaml" ] || echo "错误: Office Nav2 地图不存在: $NAV2_DIR/map/office_map.yaml"
[ -f "$NAV2_DIR/map/office_topology.yaml" ] || echo "错误: Office 拓扑地图不存在: $NAV2_DIR/map/office_topology.yaml"
[ -f "$NAV2_DIR/param/office_nav2.yaml" ] || echo "错误: Office Nav2 参数文件不存在: $NAV2_DIR/param/office_nav2.yaml"

# 检查 Gazebo 世界文件
GZ_DIR=$(ros2 pkg prefix robotics_gazebo 2>/dev/null)/share/robotics_gazebo
[ -f "$GZ_DIR/worlds/office.world" ] || echo "错误: Office Gazebo 世界文件不存在: $GZ_DIR/worlds/office.world"
```

## 3. 启动命令

### 3.1 Office 场景（基于 RMF）

Office 场景使用 RMF 进行车队管理和任务调度。

**完整 Office 仿真（Gazebo + RMF + Nav2）：**

```bash
# 无头模式（无 GUI）
ros2 launch office office.launch.xml headless:=true

# 带 Gazebo GUI
ros2 launch office office.launch.xml headless:=false

# 仅 RViz（WSL 推荐）
ros2 launch office office.launch.xml headless:=true use_rviz:=true
```

**Office 感知与建图：**

```bash
ros2 launch office office_perception_mapping.launch.xml headless:=true use_rviz:=false

# 验证 /scan 和 /map 话题
ros2 topic hz /scan
ros2 topic echo /scan sensor_msgs/msg/LaserScan --once --qos-reliability best_effort
ros2 topic echo /map nav_msgs/msg/OccupancyGrid --once
```

### 3.2 Warehouse / Office 场景（基于 Nav2）

基于 Nav2 的启动使用 `indoor_delivery.launch.py`，支持两种场景。

**无头模式 Warehouse：**

```bash
ros2 launch robotics_nav2 indoor_delivery.launch.py scene:=warehouse use_gazebo_gui:=false use_rviz:=false force_software_rendering:=true
```

**无头模式 Office：**

```bash
ros2 launch robotics_nav2 indoor_delivery.launch.py scene:=office use_gazebo_gui:=false use_rviz:=false force_software_rendering:=true
```

**仅 RViz（WSL 推荐）：**

```bash
ros2 launch robotics_nav2 indoor_delivery.launch.py scene:=office use_gazebo_gui:=false use_rviz:=true force_software_rendering:=true
```

**仅 Gazebo GUI：**

```bash
ros2 launch robotics_nav2 indoor_delivery.launch.py scene:=office use_gazebo_gui:=true use_rviz:=false force_software_rendering:=true
```

**Stage-2 演示配置：**

```bash
# Office stage-2 演示
ros2 launch robotics_nav2 indoor_delivery.launch.py \
  scene:=office \
  nav2_params_file:=$(ros2 pkg prefix robotics_nav2)/share/robotics_nav2/param/office_stage2_demo.yaml \
  use_gazebo_gui:=false use_rviz:=true force_software_rendering:=true

# Warehouse stage-2 演示
ros2 launch robotics_nav2 indoor_delivery.launch.py \
  scene:=warehouse \
  nav2_params_file:=$(ros2 pkg prefix robotics_nav2)/share/robotics_nav2/param/warehouse_stage2_demo.yaml \
  use_gazebo_gui:=false use_rviz:=true force_software_rendering:=true
```

### 3.3 WSL 启动脚本

```bash
# 仅 RViz
ROBOTICS_SCENE=office ROBOTICS_RVIZ=true ROBOTICS_GAZEBO_GUI=false \
  /path/to/Robotics_export/robotics_launcher/start_robotics_wsl.sh

# 仅 Gazebo GUI
ROBOTICS_SCENE=office ROBOTICS_RVIZ=false ROBOTICS_GAZEBO_GUI=true \
  /path/to/Robotics_export/robotics_launcher/start_robotics_wsl.sh

# Stage-2 演示
/path/to/Robotics_export/tools/run_stage2_demo_wsl.sh
SCENE=office /path/to/Robotics_export/tools/run_stage2_demo_wsl.sh
```

### 3.4 仅 Gazebo 启动（无 Nav2）

当 Nav2 不可用或只需要 3D 场景时使用：

```bash
source /opt/ros/humble/setup.bash
source ~/ros_ws/install/setup.bash

# Office 场景
ros2 launch robotics_description robotics_gazebo.launch.py \
  world:=$(ros2 pkg prefix robotics_gazebo)/share/robotics_gazebo/worlds/office.world \
  spawn_x:=55.074 spawn_y:=-58.483 \
  use_gazebo_gui:=true use_rviz:=false

# Warehouse 场景
ros2 launch robotics_description robotics_gazebo.launch.py \
  world:=$(ros2 pkg prefix robotics_gazebo)/share/robotics_gazebo/worlds/warehouse.world \
  spawn_x:=-3.071 spawn_y:=3.583 \
  use_gazebo_gui:=true use_rviz:=false
```

## 4. 任务调度

### 4.1 RMF 任务调度（Office 场景）

以下命令需要先启动 Office RMF 仿真。

```bash
# 配送
ros2 run office dispatch_delivery --use_sim_time \
  -p supplies -d hardware -ph supplies -dh hardware

# 巡检
ros2 run office dispatch_patrol --use_sim_time \
  -p charger pantry lounge hardware coe

# 清洁
ros2 run office dispatch_clean --use_sim_time

# 取消任务
ros2 run office_tasks cancel_task --use_sim_time <request_id>
```

### 4.2 基于 Nav2 的任务调度（Warehouse / Office）

```bash
# Warehouse 配送
ros2 run robotics_scenario course_task_dispatcher.py --scene warehouse --task delivery

# Warehouse 巡检
ros2 run robotics_scenario course_task_dispatcher.py --scene warehouse --task patrol

# Warehouse stage-2 演示
ros2 run robotics_scenario course_task_dispatcher.py --scene warehouse --task demo

# Office 配送
ros2 run robotics_scenario course_task_dispatcher.py --scene office --task delivery

# Office 巡检
ros2 run robotics_scenario course_task_dispatcher.py --scene office --task patrol
```

### 4.3 自然语言调度

```bash
# 关键词模式（无需 API Key）
ros2 run robotics_scenario course_nl_command.py "send the warehouse robot to complete a delivery loop"
ros2 run robotics_scenario course_nl_command.py "dispatch an office patrol route through the checkpoints"

# LLM 模式（需要 OPENAI_API_KEY）
export OPENAI_API_KEY=<key>
ros2 run robotics_scenario course_llm_command.py "send the office robot to patrol all checkpoints"
ros2 run robotics_scenario course_llm_command.py --force-fallback --dry-run "办公室巡检一圈"
```

### 4.4 Office LLM 命令（基于 config.toml）

`office_llm_command.py` 从 `config.toml` 读取 LLM 设置。若 `api_key` 为空，自动使用 mock 模式。

```bash
# Mock 模式（未配置 API Key 时的默认模式）
ros2 run office office_llm_command.py "请巡检办公室所有检查点"
ros2 run office office_llm_command.py "把文件送到硬件办公室"

# 强制 mock 模式
ros2 run office office_llm_command.py --force-mock "start an office patrol"

# 强制关键词回退
ros2 run office office_llm_command.py --force-fallback "deliver the file to the hardware office"

# 真实 OpenAI API（需要在 config.toml 配置 api_key 或设置 OPENAI_API_KEY 环境变量）
export OPENAI_API_KEY="sk-xxx"
ros2 run office office_llm_command.py "please inspect the office"

# 执行解析后的任务（非 dry-run）
ros2 run office office_llm_command.py --execute "请巡检办公室所有检查点"

# 保存结果到 JSON
ros2 run office office_llm_command.py --save-json /tmp/llm_result.json "办公室巡检"
```

### 4.5 任务调度器（多任务路线优化）

任务调度器接受多个任务，优化执行顺序以最小化总行程距离。

```bash
# Dry-run：仅显示调度计划，不实际派发
ros2 run office office_task_scheduler.py --dry-run \
  "delivery:supplies:hardware" \
  "patrol:charger,patrol_a1,patrol_a2,patrol_d1,charger" \
  "go_to_place:lounge"

# 使用 greedy TSP 策略
ros2 run office office_task_scheduler.py --dry-run --strategy greedy_tsp \
  "delivery:supplies:hardware" \
  "go_to_place:lounge" \
  "go_to_place:pantry"

# 实际派发任务
ros2 run office office_task_scheduler.py --use_sim_time \
  "delivery:supplies:hardware" \
  "go_to_place:lounge"

# 保存调度结果
ros2 run office office_task_scheduler.py --dry-run --save-json /tmp/schedule.json \
  "delivery:supplies:hardware" "go_to_place:pantry"
```

任务规格格式：
- `delivery:取货点:送货点`
- `patrol:航点1,航点2,航点3,...`
- `go_to_place:目标位置`

### 4.6 动态避障

动态避障节点监控 `/scan` 话题，检测障碍物并在路径受阻时触发重规划。

```bash
# 使用默认配置启动（从 config.toml 读取）
ros2 run office office_dynamic_obstacle_avoidance.py

# 覆盖参数
ros2 run office office_dynamic_obstacle_avoidance.py \
  --robot-name tinyRobot1 \
  --threshold 2.0 \
  --cooldown 15.0
```

### 4.7 机器人状态

```bash
# 查看机器人状态
ros2 run office office_status

# 持续监控
ros2 run office office_status --watch 1.0

# JSON 输出
ros2 run office office_status --json

# 指定机器人
ros2 run office office_status --robot tinyRobot1
```

### 4.8 实验运行器

```bash
# 生成 Nav2 参数配置
ros2 run robotics_nav2 generate_nav2_profile.py \
  --base $(ros2 pkg prefix robotics_nav2)/share/robotics_nav2/param/warehouse_nav2.yaml \
  --planner-profile navfn_astar \
  --avoidance-profile collision_monitor \
  --output /tmp/warehouse_navfn_collision.yaml

# 运行实验
ros2 run robotics_scenario course_experiment_runner.py \
  --scene warehouse \
  --task delivery \
  --trials 3 \
  --planner-profile navfn_astar \
  --avoidance-profile collision_monitor
```

## 5. 配置

### 5.1 config.toml

项目使用仓库根目录的 `config.toml` 进行集中配置，包含三个部分：

```toml
[llm]
provider = "openai"       # "openai" 或 "mock"（api_key 为空时自动设为 "mock"）
api_key = ""              # 优先使用 OPENAI_API_KEY 环境变量；留空则禁用
model = "gpt-4o-mini"     # 优先使用 OPENAI_MODEL 环境变量
base_url = "https://api.openai.com/v1"  # 优先使用 OPENAI_BASE_URL 环境变量
timeout_sec = 20

[llm.mock]
default_task = "patrol"
default_confidence = 0.80

[scheduler]
strategy = "nearest_neighbor"   # nearest_neighbor | greedy_tsp
robot_name = "tinyRobot1"
fleet_name = "tinyRobot"
default_charger = "tinyRobot1_charger"

[obstacle_avoidance]
enabled = true
scan_topic = "/scan"
robot_state_topic = "robot_state"
robot_name = "tinyRobot1"
obstacle_range_threshold = 1.5   # 米
confirm_count = 3                # 连续检测次数后触发
slowdown_factor = 0.3            # 速度乘数（0.0 = 停车）
replan_cooldown_sec = 10.0       # 重规划请求间隔（秒）
```

### 5.2 环境变量

环境变量优先级高于 `config.toml`：

| 变量 | 用途 | 默认值 |
|------|------|--------|
| `OPENAI_API_KEY` | LLM API 密钥 | （空 = mock 模式） |
| `OPENAI_MODEL` | LLM 模型名 | `gpt-4o-mini` |
| `OPENAI_BASE_URL` | LLM API 端点 | `https://api.openai.com/v1` |
| `OPENAI_PROVIDER` | 强制指定提供者 | （从 config.toml 读取） |
| `ROBOTICS_SCENE` | 启动器场景 | `office` |
| `ROBOTICS_RVIZ` | 是否启动 RViz | `false` |
| `ROBOTICS_GAZEBO_GUI` | 是否启动 Gazebo GUI | `false` |

### 5.3 Office 语义位置

Office 场景包含 13 个命名位置，用于任务调度器和派发器：

| 位置 | X | Y | 说明 |
|------|---|---|------|
| charger | 55.07 | -58.48 | 充电桩 |
| supplies | 59.68 | -31.66 | 物资间 |
| pantry | 69.81 | -93.92 | 茶水间 |
| lounge | 85.86 | -112.07 | 休息区 |
| hardware | 66.93 | -121.32 | 硬件办公室 |
| coe | 47.49 | -28.80 | COE 办公室 |
| patrol_a1 | 46.49 | -59.56 | 巡检点 A1 |
| patrol_a2 | 81.95 | -102.73 | 巡检点 A2 |
| patrol_d1 | 61.58 | -97.15 | 巡检点 D1 |
| patrol_c | 42.35 | -117.10 | 巡检点 C |
| patrol_b | 20.76 | -56.02 | 巡检点 B |
| patrol_d2 | 68.54 | -52.11 | 巡检点 D2 |
| backup_charger | 78.01 | -113.70 | 备用充电桩 |

## 6. 故障排除

### 6.1 缺少 RMF 包

**症状：** `ros2 launch office office.launch.xml` 失败，报 `Package 'office_demos' not found` 或 `ModuleNotFoundError: No module named 'rmf_adapter'`

**修复：**
```bash
sudo apt-get install -y ros-humble-rmf-dev ros-humble-rmf-fleet-adapter-python
```

### 6.2 缺少 Gazebo RMF 插件

**症状：** Gazebo 启动但机器人不动，或报错 `[Err] [FleetAdapterPlugin] Failed to load plugin`

**修复：**
```bash
sudo apt-get install -y \
  ros-humble-rmf-building-sim-gz-classic-plugins \
  ros-humble-rmf-robot-sim-gz-classic-plugins
```

### 6.3 缺少 Office 地图文件

**症状：** `office.launch.xml` 失败，报 `building_map_server` 错误或 `nav_graph_file` 未找到

**修复：** 确认 `office_maps` 已构建且生成地图存在：
```bash
ros2 pkg prefix office_maps  # 应返回路径
ls $(ros2 pkg prefix office_maps)/share/office_maps/maps/office/office.building.yaml  # 应存在
ls $(ros2 pkg prefix office_maps)/share/office_maps/generated_maps/office/  # 应包含 nav_graphs/ 和 office.world
```

如果生成地图缺失，重新构建：
```bash
colcon build --symlink-install --packages-select office_maps
```

### 6.4 Nav2 controller_server 无法启动

**症状：** `controller_server` 退出码 127，`ldd` 报 `libconversions.so` 和 `libtf_help.so` 为 `not found`

这是 apt 安装的 `ros-humble-nav2-controller` 1.1.20 `.deb` 的已知问题。

**替代方案：**
1. 不使用 Nav2 运行（仅 Gazebo 启动，见 3.4 节）
2. 从源码构建 `nav2_controller` 并覆盖工作空间
3. 改用基于 RMF 的 Office 启动（`ros2 launch office office.launch.xml`）

### 6.5 WSL 上 Gazebo 渲染问题

**症状：** `gzclient` 崩溃，报 `Failed to create OpenGL context`

**修复：**
```bash
# 检查 GPU 直通是否可用
ls /usr/lib/x86_64-linux-gnu/dri/d3d12_dri.so

# 如果 d3d12_dri.so 存在，尝试硬件渲染：
unset LIBGL_ALWAYS_SOFTWARE MESA_LOADER_DRIVER_OVERRIDE

# 如果不可用，使用软件渲染：
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_LOADER_DRIVER_OVERRIDE=llvmpipe
# 或在启动命令中添加 force_software_rendering:=true
```

### 6.6 Python BOM 错误

**症状：** Python 文件报 `SyntaxError: invalid non-printable character U+FEFF`

**修复：** 此问题已在之前的更新中修复。如果再次出现，移除受影响文件的 BOM：
```bash
find . -name "*.py" -exec sed -i '1s/^\xEF\xBB\xBF//' {} +
```

### 6.7 Fleet adapter 连接错误

**症状：** `office_fleet_adapter` 日志显示连接 `127.0.0.1:22011` 被拒绝

**修复：** Fleet manager 随 Office 启动自动运行。如果未运行：
```bash
# 检查 fleet_manager 是否运行
ros2 node list | grep fleet_manager

# 重启 Office 仿真
ros2 launch office office.launch.xml headless:=true
```

### 6.8 LLM mock 模式不工作

**症状：** `office_llm_command.py` 在没有 API Key 的情况下仍尝试调用 OpenAI API

**修复：** 确认 `config.toml` 存在于项目根目录且 `api_key` 为空：
```bash
cat config.toml | grep api_key
# 应显示: api_key = ""
```

或强制 mock 模式：
```bash
ros2 run office office_llm_command.py --force-mock "你的命令"
```

### 6.9 任务调度器报 "No module named 'rclpy'"

**症状：** `office_task_scheduler.py` 报 `ModuleNotFoundError: No module named 'rclpy'`

**修复：** 仅在 `--dry-run` 模式下如果未 source ROS 环境时出现。dry-run 模式不需要 ROS，但需要标准 Python 环境。实际派发时需要先 source ROS：
```bash
source /opt/ros/humble/setup.bash
source ~/ros_ws/install/setup.bash
```

## 7. 项目结构

```
Robotics_export/
├── config.toml                          # 全局配置（LLM、调度器、动态避障）
├── office/                              # Office 元包
│   ├── launch/                          # Office 启动文件
│   │   ├── office.launch.xml
│   │   └── office_perception_mapping.launch.xml
│   └── scripts/                         # Office 脚本
│       ├── office_llm_command.py        # LLM 命令解析（OpenAI/mock/回退）
│       ├── office_task_scheduler.py     # 多任务路线优化调度器
│       ├── office_dynamic_obstacle_avoidance.py  # 动态避障节点
│       ├── office_synthetic_lidar.py    # 合成激光雷达（/scan）
│       ├── office_scan_mapper.py        # 扫描到 OccupancyGrid 映射
│       ├── office_status                # 机器人状态监控
│       ├── dispatch_delivery            # RMF 配送调度
│       ├── dispatch_patrol              # RMF 巡检调度
│       ├── dispatch_clean               # RMF 清洁调度
│       ├── gui_delivery_test            # GUI 配送测试
│       ├── gui_patrol_test              # GUI 巡检测试
│       └── gui_clean_test               # GUI 清洁测试
├── office_tasks/                        # RMF 任务调度脚本
├── office_fleet_adapter/                # RMF 车队适配器（Python）
├── office_maps/                         # Office 导航地图与建筑数据
├── office_gz/                           # Office Gazebo 仿真启动
├── office_demos/                        # Office RMF 演示启动与配置
├── office_assets/                       # Office 3D 模型资源
├── robotics_nav2/                       # Nav2 配置与启动
│   ├── map/                             # 栅格地图 + 拓扑地图
│   ├── param/                           # Nav2 参数文件
│   └── scripts/                         # Nav2 配置生成器
├── robotics_scenario/                   # 行为树 + 场景管理器（C++）
│   ├── behavior_trees/                  # BT XML 文件
│   ├── scripts/                         # LLM/NL 命令解析器
│   └── param/                           # 语义目标、场景参数
├── robotics_interfaces/                 # 自定义 ROS 2 接口（Delivery action）
├── robotics_description/                # 机器人 URDF/xacro
├── robotics_gazebo/                     # Gazebo 世界文件
├── robotics_localization/               # EKF + HDL 定位
├── robotics_launcher/                   # tmux 启动器配置
├── tools/                               # 辅助脚本（WSL、验证）
├── docs/                                # 文档
└── experiment_results/                  # 实验数据
```

## 8. 只读工作空间（extracted-deps 运行时）

当 `/opt/`、`/usr/` 和 `/var/` 为只读时（容器、锁定镜像等），标准 `apt-get install` 流程不可用。本仓库提供了 `/tmp/sysroot` 下的解压 sysroot 以及 `scripts/` 下的环境配置文件：

```bash
# 一次性解压（仅在 /tmp/sysroot 缺失时需要）
mkdir -p /tmp/sysroot
for deb in /tmp/ros-humble-*.deb /tmp/libgazebo*.deb /tmp/gazebo*.deb; do
  dpkg-deb -x "$deb" /tmp/sysroot/ 2>/dev/null && echo "extracted: $deb"
done

# 加载运行时环境（路径、库搜索、Gazebo 资源查找）
source /home/brilliant/ros_ws/scripts/setup_robotics.sh
```

`scripts/setup_robotics.sh` 导出：

- `PATH` -> `/tmp/sysroot/usr/bin` 优先
- `LD_LIBRARY_PATH` -> `/tmp/sysroot/usr/lib/x86_64-linux-gnu` 和 `/tmp/sysroot/opt/ros/humble/lib`
- `PYTHONPATH` -> `scripts/ros_stub`（项目本地桩模块）+ 解压的 ROS Python dist-packages
- `AMENT_PREFIX_PATH` -> `/tmp/sysroot/opt/ros/humble`
- `GAZEBO_RESOURCE_PATH`、`GAZEBO_MODEL_PATH`、`GAZEBO_PLUGIN_PATH` 用于解压的 Gazebo 11
- `ROS_LOG_DIR=/tmp/ros_logs`、`ROS_HOME=/tmp/ros_home`、`HOME=/tmp`（避免只读日志目录）

项目还提供了上游缺失的 `gazebo_ros.scripts` 模块桩，位于 `scripts/ros_stub/gazebo_ros/scripts/__init__.py`。`setup_robotics.sh` 将 `scripts/ros_stub` 添加到 `PYTHONPATH`，使 `from gazebo_ros.scripts import GazeboRosPaths` 无需额外配置即可正常解析。

### WSL2 GPU 渲染（推荐）

在 WSL2 内（`IS_WSL_ENVIRONMENT=true` 且 D3D12 后端已启用 — 通过 `lspci | grep "Microsoft Corporation Device 008e"` 和 `/usr/lib/x86_64-linux-gnu/dri/d3d12_dri.so` 是否存在来验证），Gazebo 可通过 Mesa D3D12 驱动使用宿主机 GPU。去掉 `force_software_rendering:=true` 标志和两个软件渲染环境变量（`LIBGL_ALWAYS_SOFTWARE`、`MESA_LOADER_DRIVER_OVERRIDE`），让 Mesa 自动选择 D3D12 后端。

何时回退到软件渲染：

- `d3d12_dri.so` 缺失（`ls /usr/lib/x86_64-linux-gnu/dri/`）-> WSL2 GPU 直通未激活；重新添加 `force_software_rendering:=true`。
- gzclient 崩溃报 `Failed to create OpenGL context` -> Mesa/llvmpipe 有问题；运行 `LIBGL_DEBUG=verbose glxinfo -B` 检查（先安装 `mesa-utils`）。
- 在无显示的 CI 容器中运行 -> 保持 `use_gazebo_gui:=false`。

## 备注

- AWS 资产被复用于最终的 `warehouse` 世界、地图和语义路线布局。
- RMF 办公室数据被复用于最终的 `office` 地图、语义航点和巡检/配送流程。
- 运行时技术栈保持本地 ROS 2 Humble + Gazebo Classic + Nav2。
- 本导出为课程工作流自包含，不再依赖旧的 `rdsim_submodules` Git 子模块。
- CMake 包发现通过 source `/opt/ros/humble/setup.bash` 和工作空间 `install/local_setup.bash` 处理；包文件不应硬编码本地 `install/` 路径。
- 课程规划、评估、AI 使用和 sim-to-real 笔记位于 `docs/`。

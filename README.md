# Robotics

Robotics is a ROS 2 Humble course workspace that now exposes exactly two runnable scene profiles:

- `warehouse`: the AWS RoboMaker small warehouse adapted as the canonical warehouse scene
- `office`: an RMF office-inspired scene adapted to the local Gazebo Classic + Nav2 stack

## Requirements

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo 11

Recommended runtime dependencies:

```bash
sudo apt-get update && sudo apt-get install -y \
  ros-humble-robot-localization \
  ros-humble-imu-filter-madgwick \
  ros-humble-controller-manager \
  ros-humble-diff-drive-controller \
  ros-humble-interactive-marker-twist-server \
  ros-humble-joint-state-broadcaster \
  ros-humble-joint-trajectory-controller \
  ros-humble-joint-state-publisher-gui \
  ros-humble-joy \
  ros-humble-robot-state-publisher \
  ros-humble-teleop-twist-joy \
  ros-humble-twist-mux \
  ros-humble-spatio-temporal-voxel-layer \
  ros-humble-pcl-ros \
  ros-humble-pcl-conversions \
  ros-humble-rclcpp-components \
  ros-humble-xacro \
  libgazebo-dev \
  tmux \
  tmuxp
```

## WSL Quick Start

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
source ~/ros2_ws/install/local_setup.bash
```

## Launch

Headless warehouse:

```bash
ros2 launch robotics_nav2 indoor_delivery.launch.py scene:=warehouse use_gazebo_gui:=false use_rviz:=false force_software_rendering:=true
```

Headless office:

```bash
ros2 launch robotics_nav2 indoor_delivery.launch.py scene:=office use_gazebo_gui:=false use_rviz:=false force_software_rendering:=true
```

RViz navigation view only, recommended for WSL graphics:

```bash
ros2 launch robotics_nav2 indoor_delivery.launch.py scene:=office use_gazebo_gui:=false use_rviz:=true force_software_rendering:=true
```

Gazebo 3D view only:

```bash
ros2 launch robotics_nav2 indoor_delivery.launch.py scene:=office use_gazebo_gui:=true use_rviz:=false force_software_rendering:=true
```

For the warehouse scene, the Gazebo world hides the roof by default so the 3D GUI can see the interior shelves, robot, and floor from the overview camera.

WSL launcher with RViz only:

```bash
ROBOTICS_SCENE=office ROBOTICS_RVIZ=true ROBOTICS_GAZEBO_GUI=false /mnt/e/Robotic/course_robot_ws/src/Robotics_export/robotics_launcher/start_robotics_wsl.sh
```

WSL launcher with Gazebo GUI only:

```bash
ROBOTICS_SCENE=office ROBOTICS_RVIZ=false ROBOTICS_GAZEBO_GUI=true /mnt/e/Robotic/course_robot_ws/src/Robotics_export/robotics_launcher/start_robotics_wsl.sh
```

## Task Dispatch

Warehouse delivery:

```bash
ros2 run robotics_scenario course_task_dispatcher.py --scene warehouse --task delivery
```

Warehouse patrol:

```bash
ros2 run robotics_scenario course_task_dispatcher.py --scene warehouse --task patrol
```

Warehouse stage-2 demo loop:

```bash
ros2 run robotics_scenario course_task_dispatcher.py --scene warehouse --task demo
```

Office delivery:

```bash
ros2 run robotics_scenario course_task_dispatcher.py --scene office --task delivery
```

Office patrol:

```bash
ros2 run robotics_scenario course_task_dispatcher.py --scene office --task patrol
```

Natural-language dispatch:

```bash
ros2 run robotics_scenario course_nl_command.py "send the warehouse robot to complete a delivery loop"
ros2 run robotics_scenario course_nl_command.py "dispatch an office patrol route through the checkpoints"
```

LLM-backed semantic dispatch, with local keyword fallback when no API key is
available:

```bash
export OPENAI_API_KEY=<key>
ros2 run robotics_scenario course_llm_command.py "send the office robot to patrol all checkpoints"
ros2 run robotics_scenario course_llm_command.py --force-fallback --dry-run "办公室巡检一圈"
```

Stage-2 warehouse demo profile:

```bash
ros2 launch robotics_nav2 indoor_delivery.launch.py \
  scene:=warehouse \
  nav2_params_file:=$(ros2 pkg prefix robotics_nav2)/share/robotics_nav2/param/warehouse_stage2_demo.yaml \
  use_gazebo_gui:=false \
  use_rviz:=true \
  force_software_rendering:=true
```

Stage-2 office demo profile:

```bash
ros2 launch robotics_nav2 indoor_delivery.launch.py \
  scene:=office \
  nav2_params_file:=$(ros2 pkg prefix robotics_nav2)/share/robotics_nav2/param/office_stage2_demo.yaml \
  use_gazebo_gui:=false \
  use_rviz:=true \
  force_software_rendering:=true
```

Run the headless stage-2 demo script from WSL:

```bash
/mnt/e/Robotic/course_robot_ws/src/Robotics_export/tools/run_stage2_demo_wsl.sh
SCENE=office /mnt/e/Robotic/course_robot_ws/src/Robotics_export/tools/run_stage2_demo_wsl.sh
```

The `/mnt/e/...` path is only the default checkout location for this WSL setup. On another machine, set `PROJECT_ROOT=/path/to/Robotics_export` before running the helper script.

Generate planner/avoidance comparison params and record trials:

```bash
ros2 run robotics_nav2 generate_nav2_profile.py \
  --base $(ros2 pkg prefix robotics_nav2)/share/robotics_nav2/param/warehouse_nav2.yaml \
  --planner-profile navfn_astar \
  --avoidance-profile collision_monitor \
  --output /tmp/warehouse_navfn_collision.yaml

ros2 launch robotics_nav2 indoor_delivery.launch.py \
  scene:=warehouse \
  nav2_params_file:=/tmp/warehouse_navfn_collision.yaml \
  use_gazebo_gui:=false \
  use_rviz:=true \
  force_software_rendering:=true

ros2 run robotics_scenario course_experiment_runner.py \
  --scene warehouse \
  --task delivery \
  --trials 3 \
  --planner-profile navfn_astar \
  --avoidance-profile collision_monitor
```

## Notes

- AWS assets are reused for the final `warehouse` world, map, and semantic route layout.
- RMF office data is reused for the final `office` map, semantic waypoints, and patrol/delivery flow.
- The runtime stack remains local ROS 2 Humble + Gazebo Classic + Nav2.
- This export is self-contained for the course workflow and no longer depends on the old `rdsim_submodules` Git submodules.
- CMake package discovery is handled by sourcing `/opt/ros/humble/setup.bash` and the workspace `install/local_setup.bash`; package files should not hard-code a local `install/` path.
- Course planning, evaluation, AI usage, and sim-to-real notes are in `docs/`.

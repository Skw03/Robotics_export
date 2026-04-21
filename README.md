# Robotics

Robotics is a ROS 2 Humble delivery robot simulation workspace. This export keeps only the formal runtime chain, launcher scripts, and required external dependencies.

## Requirements

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo 11

Install the main runtime dependencies:

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

Set the Gazebo resource path if needed:

```bash
echo "export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11:$GAZEBO_RESOURCE_PATH" >> ~/.bashrc
source ~/.bashrc
```

## Clone And Submodules

Clone your new repository into `~/ros2_ws/src/Robotics`, then initialize submodules:

```bash
cd ~/ros2_ws/src
git clone --recursive <your-repo-url> Robotics
cd ~/ros2_ws/src/Robotics
git submodule update --init --recursive
```

## Build

Install ROS dependencies from the bundled Navigation2 source, then build the main chain:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --ignore-src --rosdistro humble --from-paths ./src/Robotics/rdsim_submodules/navigation2 -y
colcon build --symlink-install --packages-select \
  robotics_description \
  robotics_gazebo \
  robotics_interfaces \
  robotics_localization \
  robotics_nav2 \
  robotics_scenario
source ~/ros2_ws/install/local_setup.bash
```

## WSL Quick Start

If you are using the exported local workspace at `E:\Robotic\course_robot_ws\src\Robotics_export`, link it into WSL as `~/ros2_ws/src/Robotics`:

```bash
wsl -d Ubuntu-22.04
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
mkdir -p ~/ros2_ws/src
ln -sfn /mnt/e/Robotic/course_robot_ws/src/Robotics_export ~/ros2_ws/src/Robotics
source ~/ros2_ws/install/local_setup.bash
```

You can confirm the packages are visible:

```bash
ros2 pkg list | grep '^robotics_'
```

## Start Commands

Unified launcher:

```bash
bash ~/ros2_ws/src/Robotics/robotics_launcher/start_robotics_wsl.sh
```

Recommended manual startup order:

```bash
ros2 launch robotics_gazebo robotics_gazebo_world.launch.py
ros2 launch robotics_description robotics_gazebo.launch.py
ros2 launch robotics_localization hdl_localization.launch.py
ros2 launch robotics_nav2 nav2_gazebo.launch.py start_rviz:=false
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Scene Validation

Use `indoor_delivery.launch.py` for the two course scenes. It brings up Gazebo, Nav2, and `robotics_scenario` together.

Headless hotel scene launch:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/local_setup.bash
ros2 launch robotics_nav2 indoor_delivery.launch.py scene:=hotel use_gazebo_gui:=false use_rviz:=false
```

Headless warehouse scene launch:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/local_setup.bash
ros2 launch robotics_nav2 indoor_delivery.launch.py scene:=warehouse use_gazebo_gui:=false use_rviz:=false
```

Optional GUI mode in WSL, if you want to try rendering with software OpenGL:

```bash
env LIBGL_ALWAYS_SOFTWARE=1 QT_X11_NO_MITSHM=1 \
  ros2 launch robotics_nav2 indoor_delivery.launch.py scene:=hotel use_gazebo_gui:=true use_rviz:=false
```

Hotel delivery action goal:

```bash
ros2 action send_goal /hotel_delivery_scenario robotics_interfaces/action/Delivery "{
  scene_id: hotel,
  task_type: room_delivery,
  semantic_goal_id: hotel_room_101_delivery,
  start_pose: {
    header: {frame_id: map},
    pose: {position: {x: 3.5, y: 2.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}
  },
  end_pose: {
    header: {frame_id: map},
    pose: {position: {x: 4.5, y: 2.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}
  },
  return_pose: {
    header: {frame_id: map},
    pose: {position: {x: 3.5, y: 2.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}
  },
  behavior_tree: ""
}"
```

Warehouse delivery action goal:

```bash
ros2 action send_goal /warehouse_delivery_scenario robotics_interfaces/action/Delivery "{
  scene_id: warehouse,
  task_type: rack_to_dropoff,
  semantic_goal_id: warehouse_rack_A1_dropoff,
  start_pose: {
    header: {frame_id: map},
    pose: {position: {x: 3.5, y: 2.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}
  },
  end_pose: {
    header: {frame_id: map},
    pose: {position: {x: 4.5, y: 2.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}
  },
  return_pose: {
    header: {frame_id: map},
    pose: {position: {x: 3.5, y: 2.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}
  },
  behavior_tree: ""
}"
```

Acceptance criteria for both scenes:

- The launch reaches `robotics_scenario` active and the corresponding action server is available.
- `ros2 action send_goal` is accepted and returns `result: True`, `final_status: SUCCEEDED`, `task_status: COMPLETED`.
- The robot completes the three pose sequence using the scene-specific semantic goals.

Failure criteria:

- Goal rejected, scene mismatch, or BT file load failure.
- Nav2 never becomes active.
- The action returns `FAILED` or `CANCELED`.
- The robot does not complete the route within the test timeout.

Suggested metrics to record in the report:

- Task completion time.
- Action success rate across repeated runs.
- Approximate path length from odometry.
- Number of recovery behaviors triggered.
- Whether the task required a retry or manual intervention.

## Main Packages

- `robotics_description`
- `robotics_gazebo`
- `robotics_interfaces`
- `robotics_localization`
- `robotics_nav2`
- `robotics_scenario`
- `robotics_launcher`
- `rdsim_submodules`

## Known Limits

- Gazebo may still fail to create a rendering window under some WSL graphics configurations.
- RViz may still crash under some WSLg / GPU OpenGL combinations.

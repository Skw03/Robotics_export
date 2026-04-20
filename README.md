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

#!/usr/bin/env bash

set -euo pipefail

WORKSPACE_DIR="${WORKSPACE_DIR:-$HOME/ros2_ws}"
ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-humble}"
WORKSPACE_SRC_DIR="$WORKSPACE_DIR/src"
WORKSPACE_REPO_LINK="$WORKSPACE_SRC_DIR/Robotics"
LAUNCHER_DIR="$WORKSPACE_REPO_LINK/robotics_launcher"
DEFAULT_WINDOWS_REPO_PATH="/mnt/e/Robotic/course_robot_ws/src/Robotics_export"
ROBOTICS_SCENE="${ROBOTICS_SCENE:-warehouse}"
ROBOTICS_GAZEBO_GUI="${ROBOTICS_GAZEBO_GUI:-false}"
ROBOTICS_RVIZ="${ROBOTICS_RVIZ:-false}"
GAZEBO_FORCE_SOFTWARE_RENDERING="${GAZEBO_FORCE_SOFTWARE_RENDERING:-true}"
export ROBOTICS_SCENE
export ROBOTICS_GAZEBO_GUI
export ROBOTICS_RVIZ
export GAZEBO_FORCE_SOFTWARE_RENDERING

mkdir -p "$WORKSPACE_SRC_DIR"

if [[ ! -e "$WORKSPACE_REPO_LINK" ]]; then
  WINDOWS_REPO_PATH="${WINDOWS_REPO_PATH:-$DEFAULT_WINDOWS_REPO_PATH}"

  if [[ ! -d "$WINDOWS_REPO_PATH" ]]; then
    echo "Repository path not found: $WINDOWS_REPO_PATH" >&2
    echo "Expected default path: $DEFAULT_WINDOWS_REPO_PATH" >&2
    exit 1
  fi

  ln -sfn "$WINDOWS_REPO_PATH" "$WORKSPACE_REPO_LINK"
fi

set +u
source "/opt/ros/$ROS_DISTRO_NAME/setup.bash"
set -u

if [[ ! -f "$WORKSPACE_DIR/install/local_setup.bash" ]]; then
  echo "Workspace overlay not found: $WORKSPACE_DIR/install/local_setup.bash" >&2
  echo "Build the workspace first with: cd $WORKSPACE_DIR && colcon build --symlink-install" >&2
  exit 1
fi

set +u
source "$WORKSPACE_DIR/install/local_setup.bash"
set -u

if [[ ! -d "$LAUNCHER_DIR" ]]; then
  echo "Launcher directory not found: $LAUNCHER_DIR" >&2
  exit 1
fi

required_packages=(
  robotics_description
  robotics_gazebo
  robotics_interfaces
  robotics_localization
  robotics_nav2
  robotics_scenario
)

for package_name in "${required_packages[@]}"; do
  if ! ros2 pkg prefix "$package_name" >/dev/null 2>&1; then
    echo "Required package is not visible in the current environment: $package_name" >&2
    exit 1
  fi
done

cd "$LAUNCHER_DIR"
if [[ -t 1 ]]; then
  exec tmuxp load robotics_launcher.yaml
fi

exec tmuxp load -d robotics_launcher.yaml

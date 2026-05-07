#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="$HOME/ros2_ws"
SRC_LINK="$WORKSPACE/src/Robotics"
SOURCE_TREE="/mnt/e/Robotic/course_robot_ws/src/Robotics_export"
RESULT_DIR="$SOURCE_TREE/experiment_results/real_midterm"
LOG_DIR="$RESULT_DIR/logs"

mkdir -p "$WORKSPACE/src" "$RESULT_DIR" "$LOG_DIR"
ln -sfn "$SOURCE_TREE" "$SRC_LINK"

cd "$WORKSPACE"
set +u
source /opt/ros/humble/setup.bash
set -u

BUILD_LOG="$LOG_DIR/colcon_build_$(date +%Y%m%d_%H%M%S).log"
echo "[midterm] Building workspace. Log: $BUILD_LOG"
colcon build --symlink-install --packages-select \
  robotics_description \
  robotics_gazebo \
  robotics_interfaces \
  robotics_localization \
  robotics_nav2 \
  robotics_scenario 2>&1 | tee "$BUILD_LOG"

set +u
source "$WORKSPACE/install/local_setup.bash"
set -u

run_scene() {
  local scene="$1"
  local launch_log="$LOG_DIR/${scene}_launch_$(date +%Y%m%d_%H%M%S).log"
  echo "[midterm] Launching $scene. Log: $launch_log"
  ros2 launch robotics_nav2 indoor_delivery.launch.py \
    scene:="$scene" \
    use_gazebo_gui:=false \
    use_rviz:=false \
    force_software_rendering:=true >"$launch_log" 2>&1 &
  local launch_pid=$!

  cleanup_scene() {
    if kill -0 "$launch_pid" >/dev/null 2>&1; then
      echo "[midterm] Stopping $scene launch pid $launch_pid"
      pkill -TERM -P "$launch_pid" >/dev/null 2>&1 || true
      kill -TERM "$launch_pid" >/dev/null 2>&1 || true
      sleep 5
      pkill -KILL -P "$launch_pid" >/dev/null 2>&1 || true
      kill -KILL "$launch_pid" >/dev/null 2>&1 || true
    fi
    pkill -f "gzserver.*${scene}" >/dev/null 2>&1 || true
    pkill -f "gzclient.*${scene}" >/dev/null 2>&1 || true
  }
  trap cleanup_scene RETURN

  echo "[midterm] Waiting for $scene stack startup"
  sleep 115

  for task in delivery patrol; do
    echo "[midterm] Running $scene $task"
    set +e
    timeout 900 ros2 run robotics_scenario course_experiment_runner.py \
      --scene "$scene" \
      --task "$task" \
      --trials 1 \
      --planner-profile configured \
      --avoidance-profile collision_monitor \
      --output-dir "$RESULT_DIR" \
      --notes "midterm real WSL run"
    local rc=$?
    set -e
    echo "[midterm] $scene $task exit code: $rc"
  done
}

run_scene warehouse
run_scene office

echo "[midterm] Results in $RESULT_DIR"

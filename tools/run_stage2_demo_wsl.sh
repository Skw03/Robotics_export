#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-$HOME/ros2_ws}"
PROJECT_ROOT="${PROJECT_ROOT:-/mnt/e/Robotic/course_robot_ws/src/Robotics_export}"
OUTPUT_DIR="${OUTPUT_DIR:-$PROJECT_ROOT/experiment_results/stage2_demo}"
SCENE="${SCENE:-warehouse}"
LAUNCH_WAIT_SEC="${LAUNCH_WAIT_SEC:-105}"
LAUNCH_TIMEOUT_SEC="${LAUNCH_TIMEOUT_SEC:-360}"
RESULT_TIMEOUT_SEC="${RESULT_TIMEOUT_SEC:-220}"

cd "$WORKSPACE"
set +u
source /opt/ros/humble/setup.bash
source install/local_setup.bash
set -u

mkdir -p "$OUTPUT_DIR/logs"
PARAM_FILE="${PARAM_FILE:-$(ros2 pkg prefix robotics_nav2)/share/robotics_nav2/param/${SCENE}_stage2_demo.yaml}"
LAUNCH_LOG="$OUTPUT_DIR/logs/${SCENE}_stage2_demo_launch.log"

timeout "${LAUNCH_TIMEOUT_SEC}s" ros2 launch robotics_nav2 indoor_delivery.launch.py \
  scene:="$SCENE" \
  nav2_params_file:="$PARAM_FILE" \
  use_gazebo_gui:=false \
  use_rviz:=false \
  force_software_rendering:=true \
  > "$LAUNCH_LOG" 2>&1 &
LAUNCH_PID=$!

cleanup() {
  kill -INT "$LAUNCH_PID" 2>/dev/null || true
  wait "$LAUNCH_PID" 2>/dev/null || true
}
trap cleanup EXIT

sleep "$LAUNCH_WAIT_SEC"

ros2 run robotics_scenario course_experiment_runner.py \
  --scene "$SCENE" \
  --task demo \
  --trials 1 \
  --planner-profile navfn_astar \
  --avoidance-profile stage2_demo \
  --output-dir "$OUTPUT_DIR" \
  --result-timeout "$RESULT_TIMEOUT_SEC" \
  --notes "stage-2 corrected ${SCENE} demo"

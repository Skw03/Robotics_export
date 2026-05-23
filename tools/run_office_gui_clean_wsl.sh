#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-$HOME/ros2_ws}"
PROJECT_ROOT="${PROJECT_ROOT:-/mnt/e/Robotic/course_robot_ws/src/Robotics_export}"
OUTPUT_DIR="${OUTPUT_DIR:-$PROJECT_ROOT/experiment_results/office_dynamic}"
LAUNCH_WAIT_SEC="${LAUNCH_WAIT_SEC:-45}"
LAUNCH_TIMEOUT_SEC="${LAUNCH_TIMEOUT_SEC:-600}"

CLEAN_START="${CLEAN_START:-lounge}"
ROBOT="${ROBOT:-tinyRobot1}"
TRASH_ROOM="${TRASH_ROOM:-trash_room}"
CHARGER="${CHARGER:-tinyRobot1_charger}"

cd "$WORKSPACE"
set +u
source /opt/ros/humble/setup.bash
source install/local_setup.bash
set -u

mkdir -p "$OUTPUT_DIR/logs"
STAMP="$(date +%Y%m%d_%H%M%S)"
LAUNCH_LOG="$OUTPUT_DIR/logs/office_gui_clean_launch_${STAMP}.log"
CLEAN_LOG="$OUTPUT_DIR/logs/office_clean_${STAMP}.log"

timeout "${LAUNCH_TIMEOUT_SEC}s" ros2 launch office office.launch.xml   headless:=false   > "$LAUNCH_LOG" 2>&1 &
LAUNCH_PID=$!

cleanup() {
  kill -INT "$LAUNCH_PID" 2>/dev/null || true
  wait "$LAUNCH_PID" 2>/dev/null || true
}
trap cleanup EXIT

echo "Started Office GUI launch, logging to: $LAUNCH_LOG"
echo "Waiting ${LAUNCH_WAIT_SEC}s before dispatching clean task..."
sleep "$LAUNCH_WAIT_SEC"

set +e
ros2 run office dispatch_clean   --use_sim_time   -cs "$CLEAN_START"   -F tinyRobot   -R "$ROBOT"   --trash-room "$TRASH_ROOM"   --charger "$CHARGER"   > "$CLEAN_LOG" 2>&1
DISPATCH_STATUS=$?
set -e

cat "$CLEAN_LOG"

if [[ "$DISPATCH_STATUS" -ne 0 ]]; then
  echo "Clean dispatch command failed with status $DISPATCH_STATUS" >&2
  exit "$DISPATCH_STATUS"
fi

echo "Clean request sent."
echo "Launch log: $LAUNCH_LOG"
echo "Clean log: $CLEAN_LOG"
echo "Leave this script running while observing the GUI; press Ctrl+C to stop the Office demo."

wait "$LAUNCH_PID"

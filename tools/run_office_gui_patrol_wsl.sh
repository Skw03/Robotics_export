#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-$HOME/ros2_ws}"
PROJECT_ROOT="${PROJECT_ROOT:-/mnt/e/Robotic/course_robot_ws/src/Robotics_export}"
OUTPUT_DIR="${OUTPUT_DIR:-$PROJECT_ROOT/experiment_results/office_dynamic}"
LAUNCH_WAIT_SEC="${LAUNCH_WAIT_SEC:-45}"
LAUNCH_TIMEOUT_SEC="${LAUNCH_TIMEOUT_SEC:-600}"

PLACES="${PLACES:-patrol_A1 patrol_B patrol_C patrol_D1}"
ROUNDS="${ROUNDS:-3}"

cd "$WORKSPACE"
set +u
source /opt/ros/humble/setup.bash
source install/local_setup.bash
set -u

check_office_deps() {
  REQUIRED_PACKAGES=(
    rmf_task_msgs
    rmf_fleet_msgs
    rmf_dispenser_msgs
    rmf_traffic_ros2
    rmf_task_ros2
    rmf_fleet_adapter
    rmf_fleet_adapter_python
    rmf_building_map_tools
    rmf_visualization
    rmf_visualization_schedule
    rmf_lift_msgs
    ros_gz_sim
    ros_gz_bridge
    rmf_building_sim_gz_plugins
    rmf_robot_sim_gz_plugins
    teleop_twist_keyboard
    rviz2
    office
    office_gz
    office_demos
    office_maps
    office_assets
    office_fleet_adapter
    office_tasks
  )
  MISSING_PACKAGES=()
  for package in "${REQUIRED_PACKAGES[@]}"; do
    if ! ros2 pkg prefix "$package" >/dev/null 2>&1; then
      MISSING_PACKAGES+=("$package")
    fi
  done
  if [[ "${#MISSING_PACKAGES[@]}" -gt 0 ]]; then
    echo "Missing Office runtime packages:" >&2
    printf '  - %s\n' "${MISSING_PACKAGES[@]}" >&2
    exit 3
  fi
}

check_office_deps

mkdir -p "$OUTPUT_DIR/logs"
STAMP="$(date +%Y%m%d_%H%M%S)"
LAUNCH_LOG="$OUTPUT_DIR/logs/office_gui_patrol_launch_${STAMP}.log"
PATROL_LOG="$OUTPUT_DIR/logs/office_patrol_${STAMP}.log"

timeout "${LAUNCH_TIMEOUT_SEC}s" ros2 launch office office.launch.xml \
  headless:=false \
  > "$LAUNCH_LOG" 2>&1 &
LAUNCH_PID=$!

cleanup() {
  kill -INT "$LAUNCH_PID" 2>/dev/null || true
  wait "$LAUNCH_PID" 2>/dev/null || true
}
trap cleanup EXIT

echo "Started Office GUI launch, logging to: $LAUNCH_LOG"
echo "Waiting ${LAUNCH_WAIT_SEC}s before dispatching patrol..."
sleep "$LAUNCH_WAIT_SEC"

read -r -a PLACE_ARGS <<< "$PLACES"
set +e
ros2 run office dispatch_patrol \
  --use_sim_time \
  -p "${PLACE_ARGS[@]}" \
  -n "$ROUNDS" \
  > "$PATROL_LOG" 2>&1
DISPATCH_STATUS=$?
set -e

cat "$PATROL_LOG"

if [[ "$DISPATCH_STATUS" -ne 0 ]]; then
  echo "Patrol dispatch command failed with status $DISPATCH_STATUS" >&2
  exit "$DISPATCH_STATUS"
fi

echo "Patrol request sent."
echo "Launch log: $LAUNCH_LOG"
echo "Patrol log: $PATROL_LOG"
echo "Leave this script running while observing the GUI; press Ctrl+C to stop the Office demo."

wait "$LAUNCH_PID"

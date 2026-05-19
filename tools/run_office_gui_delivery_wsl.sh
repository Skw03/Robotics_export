#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-$HOME/ros2_ws}"
PROJECT_ROOT="${PROJECT_ROOT:-/mnt/e/Robotic/course_robot_ws/src/Robotics_export}"
OUTPUT_DIR="${OUTPUT_DIR:-$PROJECT_ROOT/experiment_results/office_dynamic}"
LAUNCH_WAIT_SEC="${LAUNCH_WAIT_SEC:-45}"
LAUNCH_TIMEOUT_SEC="${LAUNCH_TIMEOUT_SEC:-600}"

PICKUP="${PICKUP:-pantry}"
PICKUP_HANDLER="${PICKUP_HANDLER:-file_dispenser}"
DROPOFF="${DROPOFF:-hardware_2}"
DROPOFF_HANDLER="${DROPOFF_HANDLER:-file_ingestor}"
PAYLOAD="${PAYLOAD:-file,1}"

cd "$WORKSPACE"
set +u
source /opt/ros/humble/setup.bash
source install/local_setup.bash
set -u

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
  echo "Missing Office runtime packages (upstream Office runtime dependencies):" >&2
  printf '  - %s\n' "${MISSING_PACKAGES[@]}" >&2
  cat >&2 <<'EOF'

Install the missing ROS packages, then rebuild/source this workspace. Typical
Ubuntu 22.04 / ROS 2 Humble package names are:

sudo apt-get update && sudo apt-get install -y \
  ros-humble-rmf-task-msgs \
  ros-humble-rmf-fleet-msgs \
  ros-humble-rmf-dispenser-msgs \
  ros-humble-rmf-traffic-ros2 \
  ros-humble-rmf-task-ros2 \
  ros-humble-rmf-fleet-adapter \
  ros-humble-rmf-building-map-tools \
  ros-humble-rmf-visualization \
  ros-humble-rmf-visualization-schedule \
  ros-humble-rmf-fleet-adapter-python \
  ros-humble-rmf-lift-msgs \
  ros-humble-ros-gz-sim \
  ros-humble-ros-gz-bridge \
  ros-humble-rmf-building-sim-gz-plugins \
  ros-humble-rmf-robot-sim-gz-plugins \
  ros-humble-teleop-twist-keyboard \
  ros-humble-rviz2 \
  python3-fastapi \
  python3-flask-socketio \
  python3-pydantic \
  python3-uvicorn
EOF
  exit 3
fi

mkdir -p "$OUTPUT_DIR/logs"
STAMP="$(date +%Y%m%d_%H%M%S)"
LAUNCH_LOG="$OUTPUT_DIR/logs/office_gui_launch_${STAMP}.log"
DELIVERY_LOG="$OUTPUT_DIR/logs/office_delivery_${STAMP}.log"

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
echo "Waiting ${LAUNCH_WAIT_SEC}s before dispatching delivery..."
sleep "$LAUNCH_WAIT_SEC"

set +e
ros2 run office dispatch_delivery \
  --use_sim_time \
  -p "$PICKUP" \
  -ph "$PICKUP_HANDLER" \
  -pp "$PAYLOAD" \
  -d "$DROPOFF" \
  -dh "$DROPOFF_HANDLER" \
  -dp "$PAYLOAD" \
  > "$DELIVERY_LOG" 2>&1
DISPATCH_STATUS=$?
set -e

cat "$DELIVERY_LOG"

if [[ "$DISPATCH_STATUS" -ne 0 ]]; then
  echo "Delivery dispatch command failed with status $DISPATCH_STATUS" >&2
  exit "$DISPATCH_STATUS"
fi

echo "Delivery request sent."
echo "Launch log: $LAUNCH_LOG"
echo "Delivery log: $DELIVERY_LOG"
echo "Leave this script running while observing the GUI; press Ctrl+C to stop the Office demo."

wait "$LAUNCH_PID"

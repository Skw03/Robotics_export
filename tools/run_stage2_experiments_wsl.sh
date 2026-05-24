#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-$HOME/ros2_ws}"
PROJECT_ROOT="${PROJECT_ROOT:-/mnt/e/Robotic/course_robot_ws/src/Robotics_export}"
SCENE="${SCENE:-office}"
OUTPUT_DIR="${OUTPUT_DIR:-$PROJECT_ROOT/experiment_results/stage2_matrix}"
TRIALS="${TRIALS:-3}"
LAUNCH_WAIT_SEC="${LAUNCH_WAIT_SEC:-105}"
LAUNCH_TIMEOUT_SEC="${LAUNCH_TIMEOUT_SEC:-360}"
RESULT_TIMEOUT_SEC="${RESULT_TIMEOUT_SEC:-220}"

if [[ "$SCENE" != "office" ]]; then
  echo "This stage-2 script is locked to a single world: office"
  exit 2
fi

cd "$WORKSPACE"
set +u
source /opt/ros/humble/setup.bash
source install/local_setup.bash
set -u

mkdir -p "$OUTPUT_DIR/logs" "$OUTPUT_DIR/params"

BASE_PARAM="$(ros2 pkg prefix robotics_nav2)/share/robotics_nav2/param/office_nav2.yaml"
LAUNCH_PARAM="$OUTPUT_DIR/params/office_active.yaml"

run_case() {
  local planner_profile="$1"
  local avoidance_profile="$2"
  local case_tag="${planner_profile}_${avoidance_profile}"
  local launch_log="$OUTPUT_DIR/logs/launch_${case_tag}.log"

  ros2 run robotics_nav2 generate_nav2_profile.py \
    --base "$BASE_PARAM" \
    --planner-profile "$planner_profile" \
    --avoidance-profile "$avoidance_profile" \
    --output "$LAUNCH_PARAM"

  timeout "${LAUNCH_TIMEOUT_SEC}s" ros2 launch robotics_nav2 indoor_delivery.launch.py \
    scene:="$SCENE" \
    nav2_params_file:="$LAUNCH_PARAM" \
    use_gazebo_gui:=false \
    use_rviz:=false \
    force_software_rendering:=true \
    > "$launch_log" 2>&1 &
  local launch_pid=$!

  cleanup_case() {
    kill -INT "$launch_pid" 2>/dev/null || true
    wait "$launch_pid" 2>/dev/null || true
  }
  trap cleanup_case RETURN

  sleep "$LAUNCH_WAIT_SEC"

  ros2 run robotics_scenario course_experiment_runner.py \
    --scene "$SCENE" \
    --tasks delivery patrol \
    --trials "$TRIALS" \
    --planner-profile "$planner_profile" \
    --avoidance-profile "$avoidance_profile" \
    --result-timeout "$RESULT_TIMEOUT_SEC" \
    --output-dir "$OUTPUT_DIR" \
    --notes "stage2 single-world two-task matrix ${case_tag}"
}

run_case navfn_astar collision_monitor
run_case smac_2d collision_monitor
run_case navfn_astar baseline_costmap
run_case smac_2d baseline_costmap

echo "Experiment matrix finished. Results in: $OUTPUT_DIR"

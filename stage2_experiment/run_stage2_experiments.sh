#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-$HOME/ros2_ws}"
PROJECT_ROOT="${PROJECT_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
OUTPUT_ROOT="${OUTPUT_ROOT:-$PROJECT_ROOT/experiment_results/stage2_matrix}"

SCENES_CSV="${SCENES_CSV:-warehouse,office}"
TASKS_CSV="${TASKS_CSV:-delivery,patrol,demo}"
PLANNERS_CSV="${PLANNERS_CSV:-navfn_astar,smac_2d}"
AVOIDANCE_CSV="${AVOIDANCE_CSV:-baseline_costmap,collision_monitor}"

TRIALS="${TRIALS:-10}"
LAUNCH_WAIT_SEC="${LAUNCH_WAIT_SEC:-45}"
LAUNCH_TIMEOUT_SEC="${LAUNCH_TIMEOUT_SEC:-900}"
RESULT_TIMEOUT_SEC="${RESULT_TIMEOUT_SEC:-240}"
NEAR_COLLISION_THRESHOLD="${NEAR_COLLISION_THRESHOLD:-0.25}"

set +u
source /opt/ros/humble/setup.bash
set -u

if [[ -f "$WORKSPACE/install/local_setup.bash" ]]; then
  set +u
  source "$WORKSPACE/install/local_setup.bash"
  set -u
elif [[ -f "$PROJECT_ROOT/install/local_setup.bash" ]]; then
  set +u
  source "$PROJECT_ROOT/install/local_setup.bash"
  set -u
else
  echo "[ERROR] Cannot find install/local_setup.bash in WORKSPACE or PROJECT_ROOT."
  exit 1
fi

mkdir -p "$OUTPUT_ROOT"/{profiles,nav,logs}

IFS=',' read -r -a SCENES <<< "$SCENES_CSV"
IFS=',' read -r -a TASKS <<< "$TASKS_CSV"
IFS=',' read -r -a PLANNERS <<< "$PLANNERS_CSV"
IFS=',' read -r -a AVOIDANCES <<< "$AVOIDANCE_CSV"

launch_pid=""

cleanup_launch() {
  if [[ -n "${launch_pid}" ]]; then
    kill -INT "${launch_pid}" 2>/dev/null || true
    wait "${launch_pid}" 2>/dev/null || true
    launch_pid=""
  fi
}
trap cleanup_launch EXIT

generate_profile() {
  local scene="$1"
  local planner="$2"
  local avoidance="$3"
  local base_file
  local out_file
  base_file="$(ros2 pkg prefix robotics_nav2)/share/robotics_nav2/param/${scene}_stage2_demo.yaml"
  out_file="$OUTPUT_ROOT/profiles/${scene}_${planner}_${avoidance}.yaml"

  ros2 run robotics_nav2 generate_nav2_profile.py \
    --base "$base_file" \
    --planner-profile "$planner" \
    --avoidance-profile "$avoidance" \
    --output "$out_file"

  echo "$out_file"
}

run_combo() {
  local scene="$1"
  local planner="$2"
  local avoidance="$3"
  local profile="$4"
  local log_file="$OUTPUT_ROOT/logs/${scene}_${planner}_${avoidance}.launch.log"

  timeout "${LAUNCH_TIMEOUT_SEC}s" ros2 launch robotics_nav2 indoor_delivery.launch.py \
    scene:="$scene" \
    nav2_params_file:="$profile" \
    use_gazebo_gui:=false \
    use_rviz:=false \
    force_software_rendering:=true \
    > "$log_file" 2>&1 &
  launch_pid=$!

  sleep "$LAUNCH_WAIT_SEC"

  for task in "${TASKS[@]}"; do
    ros2 run robotics_scenario course_experiment_runner.py \
      --scene "$scene" \
      --task "$task" \
      --trials "$TRIALS" \
      --planner-profile "$planner" \
      --avoidance-profile "$avoidance" \
      --near-collision-threshold "$NEAR_COLLISION_THRESHOLD" \
      --output-dir "$OUTPUT_ROOT/nav" \
      --result-timeout "$RESULT_TIMEOUT_SEC" \
      --notes "matrix scene=${scene} planner=${planner} avoidance=${avoidance}"
  done

  cleanup_launch
}

echo "[INFO] PROJECT_ROOT=$PROJECT_ROOT"
echo "[INFO] OUTPUT_ROOT=$OUTPUT_ROOT"
echo "[INFO] SCENES=${SCENES_CSV} TASKS=${TASKS_CSV} PLANNERS=${PLANNERS_CSV} AVOIDANCE=${AVOIDANCE_CSV}"
echo "[INFO] TRIALS=${TRIALS} RESULT_TIMEOUT_SEC=${RESULT_TIMEOUT_SEC} NEAR_COLLISION_THRESHOLD=${NEAR_COLLISION_THRESHOLD}"

for scene in "${SCENES[@]}"; do
  for planner in "${PLANNERS[@]}"; do
    for avoidance in "${AVOIDANCES[@]}"; do
      profile_path="$(generate_profile "$scene" "$planner" "$avoidance")"
      echo "[INFO] Running scene=${scene}, planner=${planner}, avoidance=${avoidance}"
      run_combo "$scene" "$planner" "$avoidance" "$profile_path"
    done
  done
done

echo "[INFO] Completed. Results are under: $OUTPUT_ROOT"

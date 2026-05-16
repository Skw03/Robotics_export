#!/usr/bin/env bash
set -eo pipefail

WORKSPACE="$HOME/ros2_ws"
PROJECT_ROOT="$WORKSPACE/src/Robotics"

set +u
source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/local_setup.bash"
set -u

OUTPUT_ROOT="$PROJECT_ROOT/experiment_results/minimal_stage2"
mkdir -p "$OUTPUT_ROOT"/{profiles,nav,logs}

SCENE="warehouse"
TASK="delivery"
PLANNER="navfn_astar"
AVOIDANCE="collision_monitor"
TRIALS=2
LAUNCH_WAIT_SEC=45
RESULT_TIMEOUT_SEC=240
NEAR_COLLISION_THRESHOLD=0.25

BASE_PARAM=$(ros2 pkg prefix robotics_nav2)/share/robotics_nav2/param/${SCENE}_stage2_demo.yaml

# Generate profile
PROFILE="$OUTPUT_ROOT/profiles/${SCENE}_${PLANNER}_${AVOIDANCE}.yaml"
ros2 run robotics_nav2 generate_nav2_profile.py \
  --base "$BASE_PARAM" \
  --planner-profile "$PLANNER" \
  --avoidance-profile "$AVOIDANCE" \
  --output "$PROFILE"

echo "=== Profile settings ==="
grep -E "use_collision_detection|desired_linear_vel|planner_plugins|polygons|max_velocity" "$PROFILE"

# Launch
LOG_FILE="$OUTPUT_ROOT/logs/${SCENE}_${PLANNER}_${AVOIDANCE}.launch.log"
echo "=== Launching Gazebo + Nav2 ==="
timeout 900s ros2 launch robotics_nav2 indoor_delivery.launch.py \
  scene:="$SCENE" \
  nav2_params_file:="$PROFILE" \
  use_gazebo_gui:=false \
  use_rviz:=false \
  force_software_rendering:=true \
  > "$LOG_FILE" 2>&1 &
LAUNCH_PID=$!
echo "LAUNCH_PID=$LAUNCH_PID"

sleep "$LAUNCH_WAIT_SEC"

# Run experiment
echo "=== Running $TASK with $PLANNER + $AVOIDANCE ==="
ros2 run robotics_scenario course_experiment_runner.py \
  --scene "$SCENE" \
  --task "$TASK" \
  --trials "$TRIALS" \
  --planner-profile "$PLANNER" \
  --avoidance-profile "$AVOIDANCE" \
  --near-collision-threshold "$NEAR_COLLISION_THRESHOLD" \
  --output-dir "$OUTPUT_ROOT/nav" \
  --result-timeout "$RESULT_TIMEOUT_SEC" \
  --notes "minimal-test" 2>&1 || echo "Runner failed"

# Cleanup
kill -INT "$LAUNCH_PID" 2>/dev/null || true
wait "$LAUNCH_PID" 2>/dev/null || true

echo "=== Results ==="
for f in "$OUTPUT_ROOT/nav/"*.jsonl; do
  [ -f "$f" ] && cat "$f" || echo "No output"
done

echo "=== Launch log errors ==="
grep -E "ERROR|Goal failed|SUCCEEDED|Aborting|Failed to" "$LOG_FILE" 2>/dev/null | grep -v "Timed out" | tail -10

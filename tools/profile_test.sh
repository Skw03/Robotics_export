#!/usr/bin/env bash
set -eo pipefail

WORKSPACE="$HOME/ros2_ws"
PROJECT_ROOT="$WORKSPACE/src/Robotics"

set +u
source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/local_setup.bash"
set -u

OUTPUT_DIR="$PROJECT_ROOT/experiment_results/profile_test"
rm -rf "$OUTPUT_DIR"
mkdir -p "$OUTPUT_DIR/profiles" "$OUTPUT_DIR/nav" "$OUTPUT_DIR/logs"

BASE_PARAM=$(ros2 pkg prefix robotics_nav2)/share/robotics_nav2/param/warehouse_stage2_demo.yaml
echo "Base param: $BASE_PARAM"

PROFILE="$OUTPUT_DIR/profiles/warehouse_navfn_astar_collision_monitor.yaml"
ros2 run robotics_nav2 generate_nav2_profile.py \
  --base "$BASE_PARAM" \
  --planner-profile navfn_astar \
  --avoidance-profile collision_monitor \
  --output "$PROFILE"
echo "Generated: $PROFILE"
grep "use_collision_detection" "$PROFILE"

echo "=== Launching Gazebo + Nav2 ==="
ros2 launch robotics_nav2 indoor_delivery.launch.py \
  scene:="warehouse" \
  nav2_params_file:="$PROFILE" \
  use_gazebo_gui:=false \
  use_rviz:=false \
  force_software_rendering:=true \
  nav2_start_delay:=30.0 \
  scenario_start_delay:=75.0 \
  > "$OUTPUT_DIR/logs/launch.log" 2>&1 &
LAUNCH_PID=$!
echo "LAUNCH_PID=$LAUNCH_PID"

echo "Waiting 150s..."
sleep 150

echo "=== Running delivery trial ==="
timeout 120s ros2 run robotics_scenario course_experiment_runner.py \
  --scene warehouse \
  --task delivery \
  --trials 1 \
  --planner-profile navfn_astar \
  --avoidance-profile collision_monitor \
  --output-dir "$OUTPUT_DIR/nav" \
  --result-timeout 90 \
  --near-collision-threshold 0.25 \
  --notes "profile-test" 2>&1 || echo "Runner exit: $?"

echo "=== Launch log error summary ==="
grep -E "ERROR|SUCCEEDED|Goal failed|Aborting|collision|Recovery" "$OUTPUT_DIR/logs/launch.log" | grep -v "Timed out" | tail -10

echo "=== Results ==="
ls -la "$OUTPUT_DIR/nav/" 2>/dev/null || echo "No nav output"
for f in "$OUTPUT_DIR/nav/"*.jsonl; do
  [ -f "$f" ] && cat "$f" || true
done

kill -INT "$LAUNCH_PID" 2>/dev/null || true
wait "$LAUNCH_PID" 2>/dev/null || true
echo "=== Done ==="

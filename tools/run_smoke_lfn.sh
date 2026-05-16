#!/usr/bin/env bash
set -eo pipefail

WORKSPACE="$HOME/ros2_ws"
PROJECT_ROOT="$WORKSPACE/src/Robotics"

set +u
source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/local_setup.bash"
set -u

SCENE="warehouse"
OUTPUT_DIR="$PROJECT_ROOT/experiment_results/smoke_test"
mkdir -p "$OUTPUT_DIR/logs" "$OUTPUT_DIR/nav"

echo "=== Starting Gazebo + Nav2 ==="
ros2 launch robotics_nav2 indoor_delivery.launch.py \
  scene:="$SCENE" \
  use_gazebo_gui:=false \
  use_rviz:=false \
  force_software_rendering:=true \
  nav2_start_delay:=30.0 \
  scenario_start_delay:=75.0 \
  > "$OUTPUT_DIR/logs/smoke_launch.log" 2>&1 &
LAUNCH_PID=$!
echo "LAUNCH_PID=$LAUNCH_PID"

echo "Waiting 150s for full startup..."
sleep 150

echo "=== Checking lifecycle ==="
grep -E "lifecycle_manager.*Managed nodes|ERROR|failed" "$OUTPUT_DIR/logs/smoke_launch.log" | tail -10

echo "=== Running delivery trial ==="
timeout 120s ros2 run robotics_scenario course_experiment_runner.py \
  --scene "$SCENE" \
  --task delivery \
  --trials 1 \
  --planner-profile navfn_astar \
  --avoidance-profile collision_monitor \
  --output-dir "$OUTPUT_DIR/nav" \
  --result-timeout 90 \
  --near-collision-threshold 0.25 \
  --notes "smoke-test" 2>&1 || echo "Trial failed"

kill -INT "$LAUNCH_PID" 2>/dev/null || true
wait "$LAUNCH_PID" 2>/dev/null || true

echo "=== Results ==="
ls -la "$OUTPUT_DIR/nav/" 2>/dev/null || echo "No output"
cat "$OUTPUT_DIR/nav/"*.jsonl 2>/dev/null || echo "No jsonl"

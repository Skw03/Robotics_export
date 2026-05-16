#!/usr/bin/env bash
set -eo pipefail

WORKSPACE="$HOME/ros2_ws"
PROJECT_ROOT="$WORKSPACE/src/Robotics"

set +u
source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/local_setup.bash"
set -u

OUTPUT_DIR="$PROJECT_ROOT/experiment_results/compare_test"
mkdir -p "$OUTPUT_DIR/logs" "$OUTPUT_DIR/nav"

echo "=== Launching Gazebo + Nav2 ==="
ros2 launch robotics_nav2 indoor_delivery.launch.py \
  scene:="warehouse" \
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

for AVOID in baseline_costmap stage2_demo; do
  echo "=== Testing avoidance=$AVOID ==="
  timeout 120s ros2 run robotics_scenario course_experiment_runner.py \
    --scene warehouse \
    --task delivery \
    --trials 1 \
    --planner-profile navfn_astar \
    --avoidance-profile "$AVOID" \
    --output-dir "$OUTPUT_DIR/nav" \
    --result-timeout 90 \
    --near-collision-threshold 0.25 \
    --notes "compare-$AVOID" 2>&1
done

kill -INT "$LAUNCH_PID" 2>/dev/null || true
wait "$LAUNCH_PID" 2>/dev/null || true

echo "=== Results ==="
for f in "$OUTPUT_DIR/nav/"*.jsonl; do
  cat "$f" | python3 -c "import sys,json; d=json.loads(sys.stdin.read()); print(d.get('notes',''), d.get('status',''), d.get('task_status',''), d.get('elapsed_sec',''), d.get('path_length_m',''))" 2>/dev/null || true
done

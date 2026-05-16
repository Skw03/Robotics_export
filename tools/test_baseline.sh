#!/usr/bin/env bash
set -eo pipefail

WORKSPACE="$HOME/ros2_ws"
PROJECT_ROOT="$WORKSPACE/src/Robotics"

set +u
source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/local_setup.bash"
set -u

OUTPUT_DIR="$PROJECT_ROOT/experiment_results/baseline_test"
rm -rf "$OUTPUT_DIR"
mkdir -p "$OUTPUT_DIR/profiles" "$OUTPUT_DIR/nav" "$OUTPUT_DIR/logs"

BASE_PARAM=$(ros2 pkg prefix robotics_nav2)/share/robotics_nav2/param/warehouse_stage2_demo.yaml

# Use baseline_costmap to remove polygon monitoring + collision_monitor for comparison
for AVOID in baseline_costmap collision_monitor; do
  PROFILE="$OUTPUT_DIR/profiles/warehouse_navfn_astar_${AVOID}.yaml"
  ros2 run robotics_nav2 generate_nav2_profile.py \
    --base "$BASE_PARAM" \
    --planner-profile navfn_astar \
    --avoidance-profile "$AVOID" \
    --output "$PROFILE"

  echo "=== Profile $AVOID has: ==="
  grep "use_collision_detection\|polygons" "$PROFILE" | head -3

  echo "=== Launching with $AVOID ==="
  ros2 launch robotics_nav2 indoor_delivery.launch.py \
    scene:="warehouse" \
    nav2_params_file:="$PROFILE" \
    use_gazebo_gui:=false \
    use_rviz:=false \
    force_software_rendering:=true \
    nav2_start_delay:=30.0 \
    scenario_start_delay:=75.0 \
    > "$OUTPUT_DIR/logs/${AVOID}.log" 2>&1 &
  LAUNCH_PID=$!
  echo "LAUNCH_PID=$LAUNCH_PID"

  sleep 150

  echo "=== Running delivery trial ==="
  timeout 120s ros2 run robotics_scenario course_experiment_runner.py \
    --scene warehouse \
    --task delivery \
    --trials 1 \
    --planner-profile navfn_astar \
    --avoidance-profile "$AVOID" \
    --output-dir "$OUTPUT_DIR/nav" \
    --result-timeout 90 \
    --near-collision-threshold 0.25 \
    --notes "baseline-${AVOID}" 2>&1

  kill -INT "$LAUNCH_PID" 2>/dev/null || true
  wait "$LAUNCH_PID" 2>/dev/null || true
  sleep 5
done

echo "=== Summary ==="
for f in "$OUTPUT_DIR/nav/"*.jsonl; do
  [ -f "$f" ] && cat "$f" | python3 -c "import sys,json; d=json.loads(sys.stdin.read()); print(d.get('notes',''), d.get('status',''), d.get('task_status',''), f\"t={d.get('elapsed_sec',0):.1f}s\", f\"d={d.get('path_length_m',0):.2f}m\")" 2>/dev/null || true
done

echo "=== Errors ==="
grep -E "ERROR|polygon|Stop|Slow" "$OUTPUT_DIR/logs/"*.log 2>/dev/null | grep -v "Timed out" | tail -10

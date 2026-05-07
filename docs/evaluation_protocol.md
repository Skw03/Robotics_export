# Evaluation Protocol

## Metrics

Record the following for every trial:

- `scene`: `warehouse` or `office`
- `task`: `delivery` or `patrol`
- `planner_profile`: `navfn_astar`, `smac_2d`, or `configured`
- `avoidance_profile`: `baseline_costmap` or `collision_monitor`
- `accepted`: whether the ROS 2 action server accepted the goal
- `status` and `task_status`: final task outcome
- `elapsed_sec`: wall-clock execution time reported by the dispatcher
- `route`: semantic route used by the task
- `error`: launch, action, timeout, or navigation failure message

## Planner Comparison

Generate a profile for NavFn A*:

```bash
ros2 run robotics_nav2 generate_nav2_profile.py \
  --base $(ros2 pkg prefix robotics_nav2)/share/robotics_nav2/param/warehouse_nav2.yaml \
  --planner-profile navfn_astar \
  --avoidance-profile collision_monitor \
  --output /tmp/warehouse_navfn_collision.yaml
```

Generate a profile for Smac2D:

```bash
ros2 run robotics_nav2 generate_nav2_profile.py \
  --base $(ros2 pkg prefix robotics_nav2)/share/robotics_nav2/param/warehouse_nav2.yaml \
  --planner-profile smac_2d \
  --avoidance-profile collision_monitor \
  --output /tmp/warehouse_smac_collision.yaml
```

Launch with one generated profile at a time:

```bash
ros2 launch robotics_nav2 indoor_delivery.launch.py \
  scene:=warehouse \
  nav2_params_file:=/tmp/warehouse_navfn_collision.yaml \
  use_gazebo_gui:=false \
  use_rviz:=true \
  force_software_rendering:=true
```

Run repeated trials:

```bash
ros2 run robotics_scenario course_experiment_runner.py \
  --scene warehouse \
  --task delivery \
  --trials 3 \
  --planner-profile navfn_astar \
  --avoidance-profile collision_monitor \
  --output-dir ~/robotics_experiments
```

Repeat with `smac_2d`. Compare success rate, elapsed time, qualitative path smoothness, and observed recovery behavior.

## Avoidance Comparison

Use the same planner and compare:

- `baseline_costmap`: local/global costmap obstacle handling with collision monitor polygons disabled.
- `collision_monitor`: stop, slowdown, and footprint approach polygons enabled.

Generate a baseline profile:

```bash
ros2 run robotics_nav2 generate_nav2_profile.py \
  --base $(ros2 pkg prefix robotics_nav2)/share/robotics_nav2/param/office_nav2.yaml \
  --planner-profile navfn_astar \
  --avoidance-profile baseline_costmap \
  --output /tmp/office_navfn_baseline.yaml
```

Generate a collision-monitor profile:

```bash
ros2 run robotics_nav2 generate_nav2_profile.py \
  --base $(ros2 pkg prefix robotics_nav2)/share/robotics_nav2/param/office_nav2.yaml \
  --planner-profile navfn_astar \
  --avoidance-profile collision_monitor \
  --output /tmp/office_navfn_collision.yaml
```

## LLM Reliability Test

Run at least 10 dry-run commands and record parser, latency, confidence, and correctness:

```bash
ros2 run robotics_scenario course_llm_command.py --dry-run "send the warehouse robot from staging to outbound"
ros2 run robotics_scenario course_llm_command.py --dry-run "办公室巡检一圈"
```

Example command set:

| Command | Expected scene | Expected task |
| --- | --- | --- |
| send the warehouse robot to complete a delivery loop | warehouse | delivery |
| dispatch an office patrol route through the checkpoints | office | patrol |
| 办公室送文件到休息区 | office | delivery |
| 仓库机器人巡检货架和出货区 | warehouse | patrol |
| return the office robot after checking all checkpoints | office | patrol |
| move goods across the warehouse route | warehouse | delivery |
| 办公区补给配送 | office | delivery |
| warehouse shelf inspection loop | warehouse | patrol |
| office mail delivery route | office | delivery |
| 仓库出库配送任务 | warehouse | delivery |

## Acceptance Criteria

- At least one warehouse task and one office task complete successfully in simulation.
- Planner comparison includes at least two planner profiles on the same scene and task.
- Avoidance comparison includes baseline and collision-monitor profiles.
- LLM test table includes latency and failure cases, including API-unavailable fallback behavior.


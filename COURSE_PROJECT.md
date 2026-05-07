# ROS 2 Humble Course Project

This workspace now supports exactly two runnable scene profiles:

- `warehouse`: AWS RoboMaker small warehouse adapted to the local ROS 2 Humble stack
- `office`: RMF office-inspired office delivery and patrol scene adapted to the local Gazebo Classic + Nav2 stack

## Quick Start

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/local_setup.bash
ros2 launch robotics_nav2 indoor_delivery.launch.py scene:=warehouse use_gazebo_gui:=false use_rviz:=false
```

Dispatch a warehouse delivery task:

```bash
ros2 run robotics_scenario course_task_dispatcher.py --scene warehouse --task delivery
```

Dispatch a warehouse patrol task:

```bash
ros2 run robotics_scenario course_task_dispatcher.py --scene warehouse --task patrol
```

Dispatch an office delivery task:

```bash
ros2 run robotics_scenario course_task_dispatcher.py --scene office --task delivery
```

Dispatch an office patrol task:

```bash
ros2 run robotics_scenario course_task_dispatcher.py --scene office --task patrol
```

Dispatch from natural language:

```bash
ros2 run robotics_scenario course_nl_command.py "send the warehouse robot to complete a delivery loop"
ros2 run robotics_scenario course_nl_command.py "dispatch an office patrol route through the checkpoints"
```

Dispatch through the LLM semantic layer:

```bash
export OPENAI_API_KEY=<key>
ros2 run robotics_scenario course_llm_command.py "send the office robot to patrol all checkpoints"
ros2 run robotics_scenario course_llm_command.py --force-fallback --dry-run "办公室巡检一圈"
```

Record repeated evaluation trials:

```bash
ros2 run robotics_scenario course_experiment_runner.py \
  --scene warehouse \
  --task delivery \
  --trials 3 \
  --planner-profile navfn_astar \
  --avoidance-profile collision_monitor
```

## Course Requirement Mapping

- Occupancy-map navigation: `robotics_nav2`
- Scene/world reuse: `robotics_gazebo`
- Delivery and patrol task logic: `robotics_scenario`
- Lightweight language interface: `course_nl_command.py`
- LLM semantic planning layer: `course_llm_command.py`
- Planner comparison baseline: NavFn and Smac are both enabled in `warehouse_nav2.yaml` and `office_nav2.yaml`
- Evaluation logging: `course_experiment_runner.py` and `robotics_nav2/scripts/generate_nav2_profile.py`

## Reuse Boundaries

- AWS assets are reused for the canonical `warehouse` world, map, and semantic route layout.
- RMF demos office data is reused for the canonical `office` map, waypoint semantics, and patrol/delivery flow design.
- The runnable stack remains local ROS 2 Humble + Gazebo Classic + Nav2.

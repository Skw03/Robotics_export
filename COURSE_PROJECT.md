# ROS 2 Humble Course Project

This workspace separates the course warehouse stack from the imported upstream Office stack.

## Warehouse Course Stack

The warehouse remains the local Nav2 + Gazebo Classic runnable course scene.

```bash
ros2 launch robotics_nav2 indoor_delivery.launch.py \
  scene:=warehouse \
  use_gazebo_gui:=false \
  use_rviz:=false \
  force_software_rendering:=true
```

Warehouse tasks remain available through the course scenario dispatcher:

```bash
ros2 run robotics_scenario course_task_dispatcher.py --scene warehouse --task delivery
ros2 run robotics_scenario course_task_dispatcher.py --scene warehouse --task patrol
ros2 run robotics_scenario course_task_dispatcher.py --scene warehouse --task demo
```

## Office Stack

Office runs through the plain `office` entrypoint:

```bash
ros2 launch office office.launch.xml headless:=false
```

The old Nav2 Office scene, old Office task presets, old Office behavior trees,
old Office maps, and old Office experiment records have been removed.

## Requirement Mapping

- Warehouse occupancy-map navigation: `robotics_nav2`
- Warehouse world and assets: `robotics_gazebo`
- Warehouse delivery/patrol/demo logic: `robotics_scenario`
- Office world, navigation graph, fleet adapter, and task scripts: `office` and the imported upstream Office support packages.
- Planner comparison and stage-2 demo tooling now applies to the warehouse course stack only.

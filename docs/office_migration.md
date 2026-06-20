# Office Migration

The old course Office scene has been removed from the Nav2/Gazebo Classic
stack. Office now comes from the imported upstream Office demo stack and runs through the
plain `office` launch path in this workspace.

## Source

- Upstream source: imported Office demo stack
- Upstream commit: `e1af33e0e8619a9417bf1d23c4c05246ea1d3802`
- Imported packages:
  - `office_demos`
  - `office_gz`
  - `office_maps`
  - `office_assets`
  - `office_fleet_adapter`
  - `office_tasks`

## Launch

```bash
ros2 launch office office.launch.xml headless:=false
```

GUI delivery smoke test:

```bash
/mnt/e/Robotic/course_robot_ws/src/Robotics_export/tools/run_office_gui_delivery_wsl.sh
```

The old `scene:=office` Nav2 launch and old `--scene office` scenario
dispatcher are intentionally unavailable.

## Runtime Dependencies

The ROS environment must provide the upstream Office runtime and Gazebo Sim packages used by
the upstream demo, including traffic/task/fleet packages, visualization,
reservation node, `ros_gz_sim`, `ros_gz_bridge`, and Gazebo simulation
plugins.

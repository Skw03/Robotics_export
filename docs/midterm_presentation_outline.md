# Midterm Presentation Outline

## Slide 1: Project Title

Indoor Logistics and Patrol Mobile Robot in ROS 2 Simulation.

## Slide 2: Course Requirement Mapping

Show how the project covers perception, localization, planning, control, task logic, interaction, LLM integration, and deployment analysis.

## Slide 3: Two Application Scenes

Warehouse delivery/patrol and office delivery/patrol. Include map screenshots or RViz/Gazebo screenshots when available.

## Slide 4: System Architecture

Use the architecture diagram from `docs/option_a_system_design.md`.

## Slide 5: Robot and Sensor Model

Differential-drive base, LiDAR, camera, IMU, GPS/point cloud support, and Gazebo plugins.

## Slide 6: Nav2 Stack

AMCL, map server, static/obstacle/inflation costmaps, NavFn A*, Smac2D, Regulated Pure Pursuit, collision monitor.

## Slide 7: Task Logic

ROS 2 `Delivery.action`, semantic routes, behavior trees, and task dispatcher.

## Slide 8: LLM Semantic Layer

Natural language command to constrained JSON task selection, API mode and fallback mode.

## Slide 9: Evaluation Plan

Planner comparison, avoidance comparison, LLM reliability test, success criteria.

## Slide 10: Current Risks and Next Steps

ROS/Gazebo stability in WSL, API latency, quantitative trials, report and final demo.


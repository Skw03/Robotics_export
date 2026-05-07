# Sim-to-Real and Hardware Analysis

## Candidate Hardware Platform

The simulated robot maps to a compact differential-drive indoor mobile base:

- Differential-drive chassis with wheel odometry.
- 2D LiDAR for AMCL and obstacle layers.
- RGB or depth camera for future semantic perception.
- IMU for motion-state stabilization and diagnostics.
- Onboard compute such as Intel NUC, Jetson Orin Nano, or equivalent ROS 2-capable SBC.

## Cost and Environment Constraints

The target environment is indoor logistics or office service. The operating assumptions are flat floor, moderate speed, human-shared corridors, and payloads small enough for a tray-style platform. Outdoor weatherproofing, stairs, elevators, and heavy loads are out of scope for the current simulated prototype.

## Main Migration Barriers

| Barrier | Impact | Mitigation |
| --- | --- | --- |
| Sensor noise and field-of-view mismatch | AMCL and obstacle layers may behave differently from simulation | Calibrate LiDAR extrinsics, validate `/scan`, tune AMCL particles and costmap ranges |
| Wheel slip and actuator delay | Paths may overshoot turns or fail narrow passages | Tune velocity smoother, acceleration limits, controller frequency, and goal tolerances |
| Map mismatch | Real map may not match the ideal occupancy image | Build a real SLAM map, align semantic waypoints, add blocked-area annotations |
| Dynamic humans and carts | Static map assumptions become weaker | Increase local obstacle clearing, test collision monitor zones, add slower shared-space mode |
| Compute and network limits | LLM/API latency can delay task dispatch | Keep LLM at command parsing layer and use deterministic fallback for safety-critical execution |
| Safety certification | Simulation success does not imply physical safety | Add e-stop, speed limits, watchdogs, and human-supervised commissioning |

## Migration Strategy

1. Bring up the real robot with teleoperation, odometry, TF, LiDAR, and robot state publisher.
2. Build an occupancy map of the real indoor test area.
3. Reuse the existing Nav2 configuration as a starting point, then tune AMCL, costmaps, controller limits, and recovery behavior.
4. Port semantic waypoints from simulation to the real map.
5. Replay the same delivery and patrol tasks with low speed and manual stop supervision.
6. Keep LLM output constrained to high-level task selection; do not let model output directly control velocity commands.


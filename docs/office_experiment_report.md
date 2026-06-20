# Office Robot System Experiment Report

Generated: 2026-06-14

## 1. Experiment Overview

This report evaluates the Office robot system's dynamic obstacle avoidance and task scheduling capabilities through controlled experiments. The system is built on RMF (Robot Middleware Framework) with Gazebo simulation.

### System Architecture

```
                    +------------------+
                    |   Gazebo Sim     |
                    |  (gpu_lidar +    |
                    |   slotcar)       |
                    +--------+---------+
                             |
                    /scan (LaserScan)    robot_state (MODE_WAITING)
                             |                    |
                    +--------v--------+  +--------v---------+
                    | DynamicObstacle |  | fleet_manager    |
                    | Avoidance Node  |  | (replan=True)    |
                    | (monitor only)  |  +--------+---------+
                    +-----------------+           |
                                          +-------v---------+
                                          | RobotCommand    |
                                          | Handle.replan() |
                                          +-------+---------+
                                                  |
                                          +-------v---------+
                                          | RMF Dispatcher  |
                                          | (reroute)       |
                                          +-------+---------+
                                                  |
                                          +-------v---------+
                                          | follow_new_path |
                                          | (alternative)   |
                                          +-----------------+
```

### Key Design Decisions

| Component | Role | Mechanism |
|---|---|---|
| gpu_lidar sensor | Provides real `/scan` data | Ray-cast in Gazebo, detects walls + dynamic objects |
| slotcar plugin | Physical obstacle avoidance | `stop_distance=1.0m` auto-stop |
| DynamicObstacleAvoidance | Early warning + monitoring | Detects obstacles at 1.5m, logs events |
| fleet_manager | Replan trigger | Detects MODE_WAITING, sets replan=True |
| RobotCommandHandle | Replan execution | Polls requires_replan(), calls update_handle.replan() |
| RMF Dispatcher | Route replanning | Topology-based, finds alternative nav graph paths |

---

## 2. Experiment Design

### Experiment 1: Baseline Navigation (No Obstacles)

**Objective**: Measure baseline navigation performance without obstacles.

**Setup**:
- Robot: tinyRobot1
- Task: Patrol from `tinyRobot1_charger` to `patrol_A1`
- No obstacles on path

**Metrics**:
- Completion time
- Total distance traveled
- Moving time vs waiting time
- Success rate

### Experiment 2: Obstacle Avoidance

**Objective**: Evaluate the system's ability to handle obstacles on the robot's path.

**Setup**:
- Robot: tinyRobot1
- Task: Patrol from `tinyRobot1_charger` to `patrol_A1`
- Obstacle: 0.8m x 0.8m box placed at (12.0, -7.0) on the corridor

**Expected Behavior**:
1. gpu_lidar detects obstacle → DynamicObstacleAvoidance logs `[OBSTACLE]`
2. slotcar stops at 1.0m → MODE_WAITING
3. fleet_manager sets replan=True
4. RobotCommandHandle calls replan()
5. RMF finds alternative route → robot reroutes
6. If no alternative route → robot waits until obstacle removed

**Control Comparison**:
| Scenario | Without Replan | With Replan |
|---|---|---|
| Obstacle on path | Robot stops permanently | Robot reroutes or waits for clearance |
| Success rate | 0% (blocked) | Depends on nav graph connectivity |

### Experiment 3: Task Scheduler Optimization

**Objective**: Compare sequential vs optimized task dispatch.

**Setup**:
- Phase A: 3 tasks dispatched sequentially (original order)
- Phase B: Same 3 tasks dispatched via scheduler (nearest_neighbor optimization)

**Tasks**:
1. `delivery:supplies:hardware`
2. `go_to_place:lounge`
3. `patrol:charger,patrol_a1,patrol_a2,charger`

**Expected Improvement**:
| Metric | Sequential | Optimized | Improvement |
|---|---|---|---|
| Total distance | Higher (arbitrary order) | Lower (nearest_neighbor) | ~20-30% reduction |
| Total time | Longer | Shorter | Proportional to distance saved |

### Experiment 4: Combined System

**Objective**: Test the full system with both scheduler and obstacle avoidance.

**Setup**:
- Tasks dispatched via scheduler
- Obstacle placed on path during execution

### Experiment 5: LLM Command Parsing

**Objective**: Compare natural-language command parsing accuracy across three parser modes.

**System Architecture**:
```
User Input (natural language)
        |
        v
+------------------+
| office_llm_      |
| command.py       |
|                  |--- openai mode: OpenAI API (structured JSON output)
|                  |--- mock mode: keyword matching + mock confidence
|                  |--- keyword_fallback: pure keyword matching
+------------------+
        |
        v
  Parsed Task (delivery / patrol)
        |
        v
  dispatch_delivery / dispatch_patrol
        |
        v
  RMF Task Dispatcher -> Robot Execution
```

**Test Commands** (English + Chinese + ambiguous):

| ID | Input | Expected Task |
|---|---|---|
| patrol_en | "patrol the office corridors" | patrol |
| delivery_en | "deliver files to hardware room" | delivery |
| patrol_zh | "巡检办公室走廊" | patrol |
| delivery_zh | "把文件送到硬件区" | delivery |
| ambiguous | "check the supplies area" | patrol (default) |

**Comparison**:

| Parser | Mechanism | Expected Accuracy | Expected Latency |
|---|---|---|---|
| OpenAI (gpt-4o-mini) | LLM structured output | ~95% | 500-2000ms |
| Mock | Keyword matching + mock confidence | ~80% | <1ms |
| Keyword fallback | Pure keyword matching | ~70% | <1ms |

**How to enable OpenAI testing**:
```bash
export OPENAI_API_KEY="sk-..."
export OPENAI_BASE_URL="https://api.openai.com/v1"  # or your proxy
ros2 run office office_experiment.py --exp llm
```

Without API key, only mock and keyword_fallback are tested.

---

## 3. How to Run Experiments

### Prerequisites

```bash
# Install required packages (if not already done)
sudo apt install -y python3-socketio python3-fastapi python3-uvicorn python3-pydantic

# Build
cd ~/ros_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  office office_gz office_demos office_maps office_assets \
  office_fleet_adapter office_tasks
source install/setup.bash
```

### Step 1: Launch Simulation

```bash
# Terminal 1
ros2 launch office office_perception_mapping.launch.xml headless:=false
```

Wait for `Successfully added new robot: tinyRobot1` in the log.

### Step 2: Run Experiments

```bash
# Terminal 2
source /opt/ros/humble/setup.bash
source ~/ros_ws/install/setup.bash

# Run all experiments
ros2 run office office_experiment.py --all

# Or run individual experiments
ros2 run office office_experiment.py --exp baseline
ros2 run office office_experiment.py --exp obstacle
ros2 run office office_experiment.py --exp scheduler
ros2 run office office_experiment.py --exp llm
ros2 run office office_experiment.py --exp combined
```

### Step 3: Generate Report

```bash
# From saved data
ros2 run office office_experiment.py --report-only
```

Output files:
- `~/ros_ws/experiment_results/experiment_results.json` - Raw data
- `~/ros_ws/experiment_results/experiment_report.md` - Generated report

### Manual Obstacle Placement

If automatic obstacle spawning fails, the script will prompt you to manually place an obstacle:

1. In Gazebo, click the "Box" tool (cube icon in toolbar)
2. Click on the floor to place the box on the robot's path
3. Press Enter in the terminal to continue

---

## 4. Expected Results

### Obstacle Avoidance Effectiveness

Based on the system architecture analysis:

| Scenario | Expected Outcome | Reason |
|---|---|---|
| Obstacle on path with alternative route | Robot reroutes successfully | RMF replan finds alternative nav graph path |
| Obstacle on path without alternative route | Robot waits, then resumes when cleared | No alternative path in topology |
| Obstacle removed while robot waiting | Robot resumes original path | slotcar detects clearance, exits MODE_WAITING |
| Multiple obstacles | Sequential replanning | Each stop triggers a new replan cycle |

### Task Scheduler Efficiency

The nearest_neighbor strategy optimizes task order by:

1. Starting from the robot's current position
2. Selecting the nearest unvisited task at each step
3. This reduces backtracking compared to arbitrary order

Expected improvement depends on the spatial distribution of task locations. For the office layout:

- `tinyRobot1_charger` (10.43, -5.58)
- `patrol_A1` (12.54, -6.98)
- `patrol_A2` (15.16, -6.91)
- `supplies` (8.75, -2.27)
- `hardware_2` (5.50, -2.50)
- `lounge` (16.50, -5.50)

The nearest_neighbor strategy should reduce total distance by approximately 20-30% compared to arbitrary ordering.

---

## 5. Known Limitations

1. **Topology-based planning**: RMF uses waypoint-to-waypoint navigation, not continuous space planning. If the nav graph has only one path between two waypoints, no alternative route exists.

2. **Replan cooldown**: RobotCommandHandle has a 15-second replan cooldown. Rapid obstacle changes may not trigger immediate replanning.

3. **Monitor-only avoidance**: The DynamicObstacleAvoidance node is a monitor. It does not control the robot directly. All avoidance is handled by the slotcar plugin and RobotCommandHandle.

4. **Single-robot tracking**: The experiment script tracks only tinyRobot1 by default. Use `--robot-name tinyRobot2` for the second robot.

5. **Obstacle spawning**: Automatic obstacle spawning depends on Gazebo service availability. Manual placement may be required.

---

## 6. Files Modified

| File | Change | Purpose |
|---|---|---|
| `office/scripts/office_experiment.py` | New file | Automated experiment runner |
| `office/scripts/office_dynamic_obstacle_avoidance.py` | Rewritten | Monitor-only mode, mode tracking, stuck detection |
| `office/scripts/office_task_scheduler.py` | Modified | Waypoint name resolution, dispatch_task_request |
| `office_assets/models/TinyRobot/model.sdf` | Modified | Added gpu_lidar sensor, stop_distance=1.0m |
| `office/launch/office_perception_mapping.launch.xml` | Modified | Removed synthetic lidar (real lidar now) |
| `office/CMakeLists.txt` | Modified | Install office_experiment.py |

---

*Report template generated by office_experiment.py. Run the experiments to fill in actual data.*

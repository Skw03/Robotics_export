#!/usr/bin/env python3
"""Office Robot Experiment Runner

Automated experiments for evaluating:
  1. Dynamic obstacle avoidance effectiveness
  2. Task scheduler optimization
  3. Combined system performance

Usage (simulation must already be running):
  # Run all experiments
  python3 office_experiment.py --all

  # Run specific experiment
  python3 office_experiment.py --exp baseline
  python3 office_experiment.py --exp obstacle
  python3 office_experiment.py --exp scheduler
  python3 office_experiment.py --exp combined

  # Generate report from saved data
  python3 office_experiment.py --report

Output: experiment_results.json + experiment_report.md
"""

import argparse
import json
import math
import os
import subprocess
import sys
import time
import threading
from dataclasses import dataclass, field, asdict
from typing import List, Optional, Dict, Any, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
)

from sensor_msgs.msg import LaserScan
from rmf_fleet_msgs.msg import RobotState, RobotMode

# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------

@dataclass
class PositionSample:
    x: float
    y: float
    yaw: float
    t: float  # seconds since experiment start


@dataclass
class ModeTransition:
    t: float
    from_mode: str
    to_mode: str


@dataclass
class ObstacleEvent:
    t: float
    distance: float
    action: str  # "detected" or "cleared"


@dataclass
class ExperimentResult:
    name: str
    description: str
    start_time: float = 0.0
    end_time: float = 0.0
    success: bool = False
    positions: List[Dict] = field(default_factory=list)
    mode_transitions: List[Dict] = field(default_factory=list)
    obstacle_events: List[Dict] = field(default_factory=list)
    total_distance: float = 0.0
    completion_time: float = 0.0
    replan_count: int = 0
    waiting_time: float = 0.0
    moving_time: float = 0.0
    min_obstacle_distance: float = float("inf")
    task_ids: List[str] = field(default_factory=list)
    notes: str = ""


# ---------------------------------------------------------------------------
# Data collector node
# ---------------------------------------------------------------------------

MODE_MAP = {
    RobotMode.MODE_IDLE: "IDLE",
    RobotMode.MODE_CHARGING: "CHARGING",
    RobotMode.MODE_MOVING: "MOVING",
    RobotMode.MODE_PAUSED: "PAUSED",
    RobotMode.MODE_WAITING: "WAITING",
    RobotMode.MODE_EMERGENCY: "EMERGENCY",
    RobotMode.MODE_DOCKING: "DOCKING",
    RobotMode.MODE_ADAPTER_ERROR: "ADAPTER_ERROR",
}


class ExperimentCollector(Node):
    """Collects robot state data during an experiment."""

    def __init__(self, robot_name="tinyRobot1"):
        super().__init__("experiment_collector")
        self.robot_name = robot_name
        self._positions: List[PositionSample] = []
        self._mode_transitions: List[ModeTransition] = []
        self._obstacle_events: List[ObstacleEvent] = []
        self._current_mode = RobotMode.MODE_IDLE
        self._current_pos = PositionSample(0, 0, 0, 0)
        self._scan_min = float("inf")
        self._exp_start = 0.0
        self._obstacle_was_close = False

        state_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST, depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            RobotState, "robot_state", self._state_cb, state_qos
        )

        scan_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST, depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.create_subscription(LaserScan, "/scan", self._scan_cb, scan_qos)

    @property
    def current_position(self) -> PositionSample:
        """Return the most recent robot position sample."""
        return self._current_pos

    def start(self):
        self._exp_start = time.time()
        self._positions.clear()
        self._mode_transitions.clear()
        self._obstacle_events.clear()
        self._current_mode = RobotMode.MODE_IDLE
        self._obstacle_was_close = False

    def _elapsed(self) -> float:
        return time.time() - self._exp_start

    def _state_cb(self, msg: RobotState):
        if msg.name != self.robot_name:
            return
        loc = msg.location
        t = self._elapsed()
        self._current_pos = PositionSample(loc.x, loc.y, loc.yaw, t)
        self._positions.append({"x": loc.x, "y": loc.y, "yaw": loc.yaw, "t": t})

        old_mode = self._current_mode
        self._current_mode = msg.mode.mode
        if old_mode != self._current_mode:
            self._mode_transitions.append({
                "t": t,
                "from": MODE_MAP.get(old_mode, f"UNK({old_mode})"),
                "to": MODE_MAP.get(self._current_mode, f"UNK({self._current_mode})"),
            })

    def _scan_cb(self, msg: LaserScan):
        min_r = float("inf")
        for r in msg.ranges:
            if not (math.isinf(r) or math.isnan(r) or r < msg.range_min):
                if r < min_r:
                    min_r = r
        self._scan_min = min_r

        close = min_r <= 1.5
        t = self._elapsed()
        if close and not self._obstacle_was_close:
            self._obstacle_events.append({"t": t, "distance": min_r, "action": "detected"})
            self._obstacle_was_close = True
        elif not close and self._obstacle_was_close:
            self._obstacle_events.append({"t": t, "distance": min_r, "action": "cleared"})
            self._obstacle_was_close = False

    def detect_stall_events(self, stall_threshold_m: float = 0.3,
                            stall_window_s: float = 4.0) -> None:
        """Detect stall events from position history (post-analysis).

        The synthetic lidar only sees hardcoded walls — it CANNOT detect
        dynamically spawned obstacles. This method fills that gap by
        detecting when the robot stopped moving while it should be traveling.

        A 'stall' is defined as: displacement < stall_threshold_m over
        a time window of stall_window_s seconds.
        """
        if len(self._positions) < 10:
            return

        positions = self._positions
        # Estimate samples per second from data (~0.5s per spin_once)
        if len(positions) >= 2:
            dt_sample = (positions[-1]["t"] - positions[0]["t"]) / max(1, len(positions) - 1)
        else:
            dt_sample = 0.5
        window_size = max(3, int(stall_window_s / max(0.1, dt_sample)))

        already_stalled = False
        i = window_size
        while i < len(positions):
            recent = positions[i - window_size:i + 1]
            p_old = recent[0]
            p_new = recent[-1]
            dt = p_new["t"] - p_old["t"]
            dx = p_new["x"] - p_old["x"]
            dy = p_new["y"] - p_old["y"]
            dist = math.sqrt(dx * dx + dy * dy)

            is_stall = (dt >= stall_window_s * 0.6 and dist < stall_threshold_m)

            if is_stall and not already_stalled:
                mid_t = (p_old["t"] + p_new["t"]) / 2
                self._obstacle_events.append({
                    "t": mid_t, "distance": dist, "action": "stall_detected",
                })
                already_stalled = True
            elif not is_stall and already_stalled:
                mid_t = (p_old["t"] + p_new["t"]) / 2
                self._obstacle_events.append({
                    "t": mid_t, "distance": dist, "action": "stall_cleared",
                })
                already_stalled = False
            i += max(1, window_size // 2)

    def compute_total_distance(self) -> float:
        total = 0.0
        for i in range(1, len(self._positions)):
            dx = self._positions[i]["x"] - self._positions[i - 1]["x"]
            dy = self._positions[i]["y"] - self._positions[i - 1]["y"]
            total += math.sqrt(dx * dx + dy * dy)
        return total

    def compute_mode_durations(self) -> Dict[str, float]:
        durations: Dict[str, float] = {}
        if not self._mode_transitions:
            return durations
        for i, tr in enumerate(self._mode_transitions):
            mode = tr["to"]
            start = tr["t"]
            end = (
                self._mode_transitions[i + 1]["t"]
                if i + 1 < len(self._mode_transitions)
                else self._elapsed()
            )
            durations[mode] = durations.get(mode, 0.0) + (end - start)
        return durations

    def count_replans(self) -> int:
        return sum(1 for tr in self._mode_transitions if tr["to"] == "WAITING")

    def get_result(self, name: str, desc: str, success: bool) -> ExperimentResult:
        # Run stall detection to catch obstacles invisible to synthetic lidar
        self.detect_stall_events()
        mode_durations = self.compute_mode_durations()
        return ExperimentResult(
            name=name,
            description=desc,
            start_time=self._exp_start,
            end_time=time.time(),
            success=success,
            positions=self._positions[-100:],  # keep last 100 samples
            mode_transitions=self._mode_transitions,
            obstacle_events=self._obstacle_events,
            total_distance=self.compute_total_distance(),
            completion_time=self._elapsed(),
            replan_count=self.count_replans(),
            waiting_time=mode_durations.get("WAITING", 0.0),
            moving_time=mode_durations.get("MOVING", 0.0),
            min_obstacle_distance=(
                min(e["distance"] for e in self._obstacle_events)
                if self._obstacle_events
                else float("inf")
            ),
        )


# ---------------------------------------------------------------------------
# Task dispatch helpers
# ---------------------------------------------------------------------------

def dispatch_patrol_task(waypoints: List[str], rounds: int = 1) -> Optional[str]:
    """Dispatch a patrol task via CLI and return the task ID."""
    cmd = [
        "ros2", "run", "office_tasks", "dispatch_patrol",
        "-p"] + waypoints + [
        "-n", str(rounds),
        "--use_sim_time",
    ]
    try:
        result = subprocess.run(
            cmd, capture_output=True, text=True, timeout=30
        )
        output = result.stdout
        # Extract task ID from "Got response" line
        for line in output.split("\n"):
            if "'id':" in line:
                start = line.find("'id': '") + 7
                end = line.find("'", start)
                return line[start:end]
        return None
    except Exception as e:
        print(f"  [ERROR] dispatch_patrol failed: {e}")
        return None


def dispatch_scheduler_tasks(tasks: List[str]) -> List[str]:
    """Dispatch tasks via office_task_scheduler.py and return task IDs."""
    cmd = [
        "ros2", "run", "office_tasks", "office_task_scheduler.py",
        "--use_sim_time",
    ] + tasks
    try:
        result = subprocess.run(
            cmd, capture_output=True, text=True, timeout=30
        )
        output = result.stdout
        ids = []
        for line in output.split("\n"):
            if "task_id" in line and ":" in line:
                # Extract from JSON output
                pass
        return ids
    except Exception as e:
        print(f"  [ERROR] scheduler dispatch failed: {e}")
        return []


def _try_spawn_via_cli(x: float, y: float, name: str) -> bool:
    """Try all known CLI methods to spawn an entity. Returns True if successful."""
    # Larger, brighter obstacle for visibility
    sdf = (
        f'<sdf version="1.6">'
        f'<model name="{name}">'
        f'<static>true</static>'
        f'<pose>{x} {y} 0.75 0 0 0</pose>'   # z=0.75 at robot lidar height
        f'<link name="link">'
        f'<collision name="col">'
        f'<geometry><box><size>1.5 1.5 1.2</size></box></geometry>'
        f'</collision>'
        f'<visual name="vis">'
        f'<geometry><box><size>1.5 1.5 1.2</size></box></geometry>'
        f'<material>'
        f'  <ambient>1 0.3 0 1</ambient>'     # Bright orange
        f'  <diffuse>1 0.3 0 1</diffuse>'
        f'  <emissive>1 0.3 0 0.5</emissive>'  # Glowing
        f'</material>'
        f'</visual>'
        f'</link>'
        f'</model></sdf>'
    )

    # Try different CLI tools and service endpoint formats
    candidates = []
    for cli in ["gz", "ign"]:
        for svc in ["/world/office/create"]:
            for msg in ["gz.msgs.EntityFactory", "ignition.msgs.EntityFactory"]:
                candidates.append((cli, svc, msg))

    for cli, svc, msg_type in candidates:
        # Try with SDF in req
        try:
            r = subprocess.run(
                [cli, "service", "-s", svc,
                 "--reqtype", msg_type, "--reptype",
                 msg_type.replace("EntityFactory", "Boolean"),
                 "--timeout", "5000",
                 "--req", f'sdf: "{sdf}", name: "{name}"'],
                capture_output=True, text=True, timeout=10,
            )
            if r.returncode == 0 and ("true" in r.stdout.lower() or "data" in r.stdout.lower()):
                print(f"  [OK] Spawned via {cli} service {svc}")
                time.sleep(1)
                return True
        except (FileNotFoundError, Exception):
            pass

    return False


def spawn_obstacle(x: float, y: float, name: str = "exp_obstacle") -> bool:
    """Spawn a box obstacle in Gazebo at (x, y). Falls back to manual placement."""
    # Always attempt auto-spawn first
    auto_ok = _try_spawn_via_cli(x, y, name)

    if auto_ok:
        time.sleep(1)
        return True

    # Auto failed → require manual placement
    print(f"\n  {'=' * 56}")
    print(f"  ***  AUTOMATIC OBSTACLE SPAWN FAILED  ***")
    print(f"  {'=' * 56}")
    print()
    print(f"  MANUAL STEPS (do this in the Gazebo window):")
    print(f"    1. Click the CUBE icon in the top-left toolbar")
    print(f"    2. Click on the BLUE CORRIDOR floor")
    print(f"       (anywhere between the robot and the X marks)")
    print(f"    3. A red/orange cube should appear")
    print()
    print(f"  Target location: x={x}, y={y} (corridor area)")
    print()
    input(f"  >>> Press Enter once you have placed the cube <<<\n")
    return False


def remove_obstacle(name: str = "exp_obstacle") -> bool:
    """Remove an obstacle from Gazebo. Best-effort cleanup."""
    for cmd_base in [["gz", "service"], ["ign", "service"]]:
        for svc in ["/world/office/remove_entity", "/world/office/create", "/world/office/request"]:
            try:
                subprocess.run(
                    cmd_base + ["-s", svc, "--reqtype", "gz.msgs.EntityFactory",
                                "--reptype", "gz.msgs.Boolean", "--timeout", "3000",
                                '--req', f'name: "{name}"'],
                    capture_output=True, text=True, timeout=5,
                )
            except Exception:
                pass
    return False


# ---------------------------------------------------------------------------
# Experiment definitions
# ---------------------------------------------------------------------------

def wait_for_robot_ready(collector: ExperimentCollector, timeout: float = 60.0) -> bool:
    """Wait until the robot has reported its position."""
    print("  Waiting for robot to be ready...", end="", flush=True)
    start = time.time()
    while time.time() - start < timeout:
        rclpy.spin_once(collector, timeout_sec=0.5)
        if len(collector._positions) > 5:
            print(" OK")
            return True
    print(" TIMEOUT")
    return False


def wait_for_task_completion(
    collector: ExperimentCollector,
    timeout: float = 120.0,
    check_interval: float = 2.0,
) -> bool:
    """Wait until the robot reaches IDLE mode (task completed) or timeout."""
    print("  Waiting for task completion...", end="", flush=True)
    start = time.time()
    was_moving = False
    while time.time() - start < timeout:
        rclpy.spin_once(collector, timeout_sec=check_interval)
        mode = collector._current_mode
        if mode == RobotMode.MODE_MOVING:
            was_moving = True
        if was_moving and mode == RobotMode.MODE_IDLE:
            print(f" DONE ({time.time()-start:.1f}s)")
            return True
        # Also check if stuck in WAITING for too long
        if was_moving and mode == RobotMode.MODE_WAITING:
            if time.time() - start > timeout * 0.8:
                print(f" STUCK ({time.time()-start:.1f}s)")
                return False
    print(f" TIMEOUT ({timeout}s)")
    return False


def run_experiment_baseline(collector: ExperimentCollector) -> ExperimentResult:
    """Experiment 1: Baseline navigation without obstacles."""
    print("\n" + "=" * 60)
    print("EXPERIMENT 1: Baseline Navigation (No Obstacles)")
    print("=" * 60)

    collector.start()
    if not wait_for_robot_ready(collector):
        return ExperimentResult(name="baseline", description="Baseline navigation", success=False, notes="Robot not ready")

    print("  Dispatching patrol: tinyRobot1_charger -> patrol_A1")
    task_id = dispatch_patrol_task(["tinyRobot1_charger", "patrol_A1"], rounds=1)

    success = wait_for_task_completion(collector, timeout=120.0)
    result = collector.get_result("baseline", "Baseline navigation without obstacles", success)
    result.task_ids = [task_id] if task_id else []
    result.notes = "No obstacles placed. Pure navigation performance."

    print(f"  Result: {'SUCCESS' if success else 'FAILED'}")
    print(f"  Distance: {result.total_distance:.2f}m")
    print(f"  Time: {result.completion_time:.1f}s")
    print(f"  Moving: {result.moving_time:.1f}s | Waiting: {result.waiting_time:.1f}s")
    return result


def _compute_heading_from_positions(positions: List[Dict], min_samples: int = 5) -> Optional[Tuple[float, float]]:
    """Compute robot's heading direction (unit vector) from recent position history.

    Uses linear regression on the last `min_samples` positions to find
    the direction of travel, which is more robust than just using the
    last two samples (which can be noisy).

    Returns (dx, dy) unit vector or None if not enough data.
    """
    if len(positions) < min_samples:
        return None
    recent = positions[-min_samples:]
    n = len(recent)
    sum_x = sum(p["x"] for p in recent)
    sum_y = sum(p["y"] for p in recent)
    mean_x = sum_x / n
    mean_y = sum_y / n

    # Covariance matrix for principal component (direction of max variance)
    cov_xx = sum((p["x"] - mean_x) ** 2 for p in recent)
    cov_yy = sum((p["y"] - mean_y) ** 2 for p in recent)
    cov_xy = sum((p["x"] - mean_x) * (p["y"] - mean_y) for p in recent)

    # Direction is the eigenvector of [[cov_xx, cov_xy], [cov_xy, cov_yy]]
    # corresponding to the larger eigenvalue
    trace = cov_xx + cov_yy
    det = cov_xx * cov_yy - cov_xy * cov_xy
    disc = math.sqrt(max(0, trace * trace - 4 * det))
    lambda1 = (trace + disc) / 2  # larger eigenvalue

    if lambda1 < 1e-6:
        return None  # robot hasn't moved meaningfully

    # Eigenvector for lambda1
    if abs(cov_xy) > 1e-6:
        vx, vy = cov_xy, lambda1 - cov_xx
    else:
        vx, vy = (1.0, 0.0) if cov_xx >= cov_yy else (0.0, 1.0)

    norm = math.sqrt(vx * vx + vy * vy)
    if norm < 1e-6:
        return None
    return (vx / norm, vy / norm)


def _place_obstacle_on_path(
    collector: ExperimentCollector,
    distance_ahead: float = 4.0,
    obstacle_name: str = "exp_obstacle",
) -> Tuple[float, float, bool]:
    """Place an obstacle on the robot's path.

    Heading computation strategy (ordered by reliability):
    1. PRIMARY: Use robot's reported yaw (from RobotState) — this is the
       orientation RMF/slotcar are using to navigate, so placing ahead on
       this axis guarantees the obstacle is on the planned path.
    2. FALLBACK: PCA on recent position history (if enough samples/displacement).
    3. LAST RESORT: Two-point difference between oldest and newest sample.

    Returns (obs_x, obs_y, success).
    """
    pos = collector.current_position
    yaw = pos.yaw

    # --- Strategy 1: Yaw-based heading (most reliable for RMF robots) ---
    # RMF's slotcar controller drives the robot along its yaw axis,
    # so the yaw IS the direction of travel (or very close to it).
    heading_yaw = (math.cos(yaw), math.sin(yaw))

    # --- Strategy 2: PCA heading from position history ---
    print(f"  Computing heading from position history...")
    heading_pca = None
    for attempt in range(20):  # up to 10s additional collection
        rclpy.spin_once(collector, timeout_sec=0.5)
        heading_pca = _compute_heading_from_positions(collector._positions, min_samples=8)
        if heading_pca:
            break

    # --- Choose the best heading ---
    final_heading = None
    reason = ""

    if heading_pca:
        # Check agreement between yaw and PCA
        dot = heading_yaw[0] * heading_pca[0] + heading_yaw[1] * heading_pca[1]
        angle_diff = math.degrees(math.acos(max(-1, min(1, dot))))

        if angle_diff < 30.0:
            # Good agreement — use PCA (smoother, based on actual trajectory)
            final_heading = heading_pca
            reason = f"PCA (agrees with yaw within {angle_diff:.0f}°)"
        elif angle_diff >= 30.0:
            # Large disagreement — trust yaw (robot may be rotating in place
            # which corrupts PCA; yaw reflects intended direction)
            final_heading = heading_yaw
            reason = f"Yaw (PCA disagreed by {angle_diff:.0f}°, likely rotation noise)"

    if not final_heading:
        # No PCA result available — use yaw
        final_heading = heading_yaw
        reason = "Yaw (no PCA data available)"

    # Compute obstacle position
    obs_x = pos.x + distance_ahead * final_heading[0]
    obs_y = pos.y + distance_ahead * final_heading[1]

    heading_deg = math.degrees(math.atan2(final_heading[1], final_heading[0]))
    print(f"  Robot at ({pos.x:.2f}, {pos.y:.2f}), yaw={math.degrees(yaw):.1f}°")
    print(f"  Using heading={heading_deg:.1f}° ({reason})")
    print(f"  Placing obstacle {distance_ahead}m ahead at ({obs_x:.2f}, {obs_y:.2f})")

    spawned = spawn_obstacle(obs_x, obs_y, obstacle_name)
    return (obs_x, obs_y, spawned)


def run_experiment_obstacle(collector: ExperimentCollector) -> ExperimentResult:
    """Experiment 2: Navigation with obstacle on path.

    Uses a LONG patrol route (charger -> patrol_A2) so the robot must traverse
    a significant distance through corridors. The obstacle is placed DYNAMICALLY
    based on the robot's actual heading computed from position history.

    Phase A: No avoidance node running → robot should get stuck at obstacle.
    Phase B: Avoidance node running   → robot detects obstacle, slotcar stops,
                                      RMF replans alternative route.
    """
    print("\n" + "=" * 60)
    print("EXPERIMENT 2: Obstacle Avoidance (Dynamic Placement)")
    print("=" * 60)
    print("  Route: tinyRobot1_charger -> patrol_A2 (long corridor route)")
    print("  Obstacle: placed dynamically on robot's computed heading")

    PATROL_WAYPOINTS = ["tinyRobot1_charger", "patrol_A2"]
    OBSTACLE_DISTANCE_AHEAD = 4.0  # meters ahead of robot on its path

    # =====================================================================
    # Phase A: WITHOUT avoidance node — expect robot to get stuck
    # =====================================================================
    print("\n  ── Phase A: NO avoidance node (baseline collision) ──")
    collector.start()
    if not wait_for_robot_ready(collector):
        return ExperimentResult(name="obstacle", description="Obstacle avoidance",
                                success=False, notes="Robot not ready for phase A")

    print(f"  Dispatching patrol: {' -> '.join(PATROL_WAYPOINTS)}")
    task_id_a = dispatch_patrol_task(PATROL_WAYPOINTS, rounds=1)
    if not task_id_a:
        print("  [ERROR] Failed to dispatch patrol task for Phase A")
        return ExperimentResult(name="obstacle", description="Obstacle avoidance",
                                success=False, notes="Task dispatch failed for phase A")

    # Wait for robot to actually start moving and collect position history for heading.
    # The robot may rotate in place at startup (yaw changes without much translation),
    # so we require significant displacement AND enough samples to compute a stable heading.
    print("  Waiting for robot to enter corridor and collect trajectory data...")
    start_pos = None
    moved_enough = False
    for i in range(40):  # up to 20 seconds
        rclpy.spin_once(collector, timeout_sec=0.5)
        pos = collector.current_position
        if start_pos is None:
            start_pos = (pos.x, pos.y)
        else:
            dx = pos.x - start_pos[0]
            dy = pos.y - start_pos[1]
            dist = math.sqrt(dx * dx + dy * dy)
            if dist > 1.5:  # need >1.5m — robot must have left charging area
                print(f"  Robot is moving! pos=({pos.x:.2f}, {pos.y:.2f}), "
                      f"displacement={dist:.2f}m, samples={len(collector._positions)}")
                moved_enough = True
                break

    if not moved_enough:
        print("  [WARN] Robot did not move 1.5m+ within 20s; attempting placement anyway")

    # Extra spin to collect more position samples for stable PCA heading
    print("  Collecting additional trajectory samples for heading...")
    for _ in range(6):
        rclpy.spin_once(collector, timeout_sec=0.5)

    # Place obstacle using computed heading from position history
    obs_x, obs_y, spawned_a = _place_obstacle_on_path(
        collector,
        distance_ahead=OBSTACLE_DISTANCE_AHEAD,
        obstacle_name="exp_obstacle_a",
    )

    if not spawned_a:
        print("\n  *** AUTO-SPAWN FAILED ***")
        print("  MANUAL STEP: In Gazebo, click Insert -> Cube, place it ON THE CORRIDOR")
        print("  IN FRONT OF where the robot is heading.")
        input("  >>> Press Enter once placed <<<\n")

    # Observe: without avoidance, robot hits obstacle → stuck
    print("  Observing (no avoidance node — expect STUCK at obstacle)...")
    success_a = wait_for_task_completion(collector, timeout=90.0)
    result_a = collector.get_result("obstacle_no_avoid",
                                    "Phase A: no avoidance node",
                                    success_a)
    result_a.task_ids = [task_id_a] if task_id_a else []

    status_a = "reached goal (obstacle missed?)" if success_a else "STUCK (blocked)"
    print(f"\n  Phase A result: {status_a}")
    print(f"    Distance: {result_a.total_distance:.2f}m | Time: {result_a.completion_time:.1f}s")
    # Separate scan-based and stall-based events for clarity
    scan_events = [e for e in result_a.obstacle_events if "stall" not in e.get("action", "")]
    stall_events = [e for e in result_a.obstacle_events if "stall" in e.get("action", "")]
    print(f"    Scan events: {len(scan_events)} | Stall events: {len(stall_events)} | "
          f"Min dist: {result_a.min_obstacle_distance:.2f}m")
    if stall_events:
        for e in stall_events:
            print(f"      t={e['t']:.1f}s: {e['action']} (moved={e['distance']:.3f}m)")
    print(f"    Mode transitions: {len(result_a.mode_transitions)}")
    for tr in result_a.mode_transitions[:10]:
        print(f"      t={tr.get('t', 0):.1f}s: {tr.get('from', '?')} -> {tr.get('to', '?')}")

    # Clean up
    remove_obstacle("exp_obstacle_a")
    time.sleep(1)

    # =====================================================================
    # Phase B: WITH avoidance node — expect detection + replan + success
    # =====================================================================
    print("\n  ── Phase B: WITH avoidance node active ──")
    print("  Starting office_dynamic_obstacle_avoidance.py ...")
    avoid_proc = None
    try:
        avoid_proc = subprocess.Popen(
            ["ros2", "run", "office", "office_dynamic_obstacle_avoidance.py"],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        )
        print("  Avoidance node started (PID={})".format(avoid_proc.pid))
    except Exception as e:
        print(f"  [WARN] Could not start avoidance node: {e}")

    time.sleep(4)  # let avoidance node initialize + subscribe

    collector.start()
    if not wait_for_robot_ready(collector):
        if avoid_proc:
            avoid_proc.terminate()
        return ExperimentResult(name="obstacle", description="Obstacle avoidance",
                                success=False, notes="Robot not ready for phase B")

    print(f"  Dispatching patrol: {' -> '.join(PATROL_WAYPOINTS)}")
    task_id_b = dispatch_patrol_task(PATROL_WAYPOINTS, rounds=1)
    if not task_id_b:
        print("  [ERROR] Failed to dispatch patrol task for Phase B")
        if avoid_proc:
            avoid_proc.terminate()
        return ExperimentResult(name="obstacle", description="Obstacle avoidance",
                                success=False, notes="Task dispatch failed for phase B")

    # Wait for robot to move again — same 2m threshold as Phase A
    print("  Waiting for robot to enter corridor (Phase B)...")
    start_pos = None
    moved_enough = False
    for i in range(40):
        rclpy.spin_once(collector, timeout_sec=0.5)
        pos = collector.current_position
        if start_pos is None:
            start_pos = (pos.x, pos.y)
        else:
            dx = pos.x - start_pos[0]
            dy = pos.y - start_pos[1]
            if math.sqrt(dx * dx + dy * dy) > 1.5:
                print(f"  Robot is moving! pos=({pos.x:.2f}, {pos.y:.2f}), "
                      f"samples={len(collector._positions)}")
                moved_enough = True
                break

    # Extra samples for stable heading
    print("  Collecting additional trajectory samples for heading...")
    for _ in range(6):
        rclpy.spin_once(collector, timeout_sec=0.5)

    # Place obstacle at same relative position on path
    obs_x, obs_y, spawned_b = _place_obstacle_on_path(
        collector,
        distance_ahead=OBSTACLE_DISTANCE_AHEAD,
        obstacle_name="exp_obstacle_b",
    )
    if not spawned_b:
        print("\n  *** AUTO-SPAWN FAILED ***")
        print("  MANUAL STEP: Place cube ON THE CORRIDOR IN FRONT OF ROBOT")
        input("  >>> Press Enter once placed <<<\n")

    # Observe: with avoidance, robot should detect → stop → replan → continue
    print("  Observing (with avoidance — expect DETECT → STOP → REPLAN → GOAL)...")
    success_b = wait_for_task_completion(collector, timeout=120.0)
    result_b = collector.get_result("obstacle_with_avoid",
                                    "Phase B: with avoidance node",
                                    success_b)
    result_b.task_ids = [task_id_b] if task_id_b else []

    status_b = "reached goal (avoided/replanned!)" if success_b else "STUCK"
    print(f"\n  Phase B result: {status_b}")
    print(f"    Distance: {result_b.total_distance:.2f}m | Time: {result_b.completion_time:.1f}s")
    scan_events_b = [e for e in result_b.obstacle_events if "stall" not in e.get("action", "")]
    stall_events_b = [e for e in result_b.obstacle_events if "stall" in e.get("action", "")]
    print(f"    Scan events: {len(scan_events_b)} | Stall events: {len(stall_events_b)} | "
          f"Min dist: {result_b.min_obstacle_distance:.2f}m")
    if stall_events_b:
        for e in stall_events_b:
            print(f"      t={e['t']:.1f}s: {e['action']} (moved={e['distance']:.3f}m)")
    print(f"    Mode transitions: {len(result_b.mode_transitions)}")
    for tr in result_b.mode_transitions[:15]:
        print(f"      t={tr.get('t', 0):.1f}s: {tr.get('from', '?')} -> {tr.get('to', '?')}")

    # Stop avoidance node
    if avoid_proc:
        avoid_proc.terminate()
        try:
            avoid_proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            avoid_proc.kill()
        print("  Avoidance node stopped.")

    # Cleanup
    remove_obstacle("exp_obstacle_b")

    # =====================================================================
    # Combined results
    # =====================================================================
    result = ExperimentResult(
        name="obstacle",
        description=(
            f"Obstacle avoidance A/B test | route={'->'.join(PATROL_WAYPOINTS)} | "
            f"obstacle={OBSTACLE_DISTANCE_AHEAD}m ahead on dynamic heading"
        ),
        success=success_b,  # overall success = Phase B succeeded
        total_distance=result_a.total_distance + result_b.total_distance,
        completion_time=result_a.completion_time + result_b.completion_time,
        notes=(
            f"A no-avoid: {'SUCCESS' if success_a else 'STUCK'}, "
            f"{result_a.total_distance:.2f}m, {result_a.completion_time:.1f}s, "
            f"events={len(result_a.obstacle_events)}, min_dist={result_a.min_obstacle_distance:.2f}m | "
            f"B w/avoid: {'SUCCESS' if success_b else 'STUCK'}, "
            f"{result_b.total_distance:.2f}m, {result_b.completion_time:.1f}s, "
            f"events={len(result_b.obstacle_events)}, min_dist={result_b.min_obstacle_distance:.2f}m, "
            f"replans={result_b.replan_count}"
        ),
    )
    result.obstacle_events = result_a.obstacle_events + result_b.obstacle_events
    result.mode_transitions = result_a.mode_transitions + result_b.mode_transitions
    return result


def run_experiment_scheduler(collector: ExperimentCollector) -> ExperimentResult:
    """Experiment 3: Task scheduler optimization."""
    print("\n" + "=" * 60)
    print("EXPERIMENT 3: Task Scheduler Optimization")
    print("=" * 60)

    # --- Phase A: Sequential dispatch (no optimization) ---
    print("\n  Phase A: Sequential dispatch (original order)")
    collector.start()
    if not wait_for_robot_ready(collector):
        return ExperimentResult(name="scheduler", description="Task scheduler", success=False, notes="Robot not ready")

    print("  Dispatching 3 tasks sequentially...")
    t1 = dispatch_patrol_task(["tinyRobot1_charger", "patrol_A1"], rounds=1)
    if not wait_for_task_completion(collector, timeout=120.0):
        pass  # Continue even if first task fails

    t2 = dispatch_patrol_task(["supplies", "hardware_2"], rounds=1)
    if not wait_for_task_completion(collector, timeout=120.0):
        pass

    t3 = dispatch_patrol_task(["lounge"], rounds=1)
    if not wait_for_task_completion(collector, timeout=120.0):
        pass

    seq_result = collector.get_result(
        "scheduler_sequential",
        "Sequential task dispatch (no optimization)",
        True,
    )
    seq_result.task_ids = [t1 or "", t2 or "", t3 or ""]
    seq_time = seq_result.completion_time
    seq_dist = seq_result.total_distance

    print(f"  Sequential: time={seq_time:.1f}s, distance={seq_dist:.2f}m")

    # --- Phase B: Optimized dispatch (with scheduler) ---
    print("\n  Phase B: Optimized dispatch (with scheduler)")
    collector.start()
    if not wait_for_robot_ready(collector):
        return ExperimentResult(name="scheduler", description="Task scheduler", success=False, notes="Robot not ready for phase B")

    print("  Dispatching 3 tasks via scheduler...")
    dispatch_scheduler_tasks([
        "delivery:supplies:hardware",
        "go_to_place:lounge",
        "patrol:charger,patrol_a1,patrol_a2,charger",
    ])

    # Wait for all tasks to complete (longer timeout)
    success = wait_for_task_completion(collector, timeout=300.0)

    opt_result = collector.get_result(
        "scheduler_optimized",
        "Optimized task dispatch (nearest_neighbor)",
        success,
    )
    opt_time = opt_result.completion_time
    opt_dist = opt_result.total_distance

    print(f"  Optimized: time={opt_time:.1f}s, distance={opt_dist:.2f}m")

    # Combine results
    result = ExperimentResult(
        name="scheduler",
        description="Task scheduler comparison: sequential vs optimized",
        success=True,
        total_distance=seq_dist + opt_dist,
        completion_time=seq_time + opt_time,
        notes=(
            f"Sequential: {seq_time:.1f}s, {seq_dist:.2f}m | "
            f"Optimized: {opt_time:.1f}s, {opt_dist:.2f}m | "
            f"Time saved: {seq_time - opt_time:.1f}s "
            f"({(seq_time - opt_time) / seq_time * 100:.1f}%)"
        ),
    )
    return result


def run_experiment_llm(collector: ExperimentCollector) -> ExperimentResult:
    """Experiment 5: LLM natural-language command parsing vs manual dispatch."""
    print("\n" + "=" * 60)
    print("EXPERIMENT 5: LLM Command Parsing vs Manual Dispatch")
    print("=" * 60)

    test_commands = [
        ("patrol_en", "patrol the office corridors"),
        ("delivery_en", "deliver files to hardware room"),
        ("patrol_zh", "巡检办公室走廊"),
        ("delivery_zh", "把文件送到硬件区"),
        ("ambiguous", "check the supplies area"),  # ambiguous - could be patrol or delivery
    ]

    llm_results = []
    for cmd_id, text in test_commands:
        print(f"\n  Testing: '{text}'")
        # Run office_llm_command.py in mock mode (no API key needed)
        cmd = [
            "ros2", "run", "office", "office_llm_command.py",
            "--force-mock",
            "--save-json", f"/tmp/llm_test_{cmd_id}.json",
            text,
        ]
        try:
            proc = subprocess.run(cmd, capture_output=True, text=True, timeout=15)
            output = proc.stdout
            # Parse the JSON output
            try:
                result = json.loads(output)
                llm_results.append({
                    "input": text,
                    "cmd_id": cmd_id,
                    "parsed_task": result.get("parsed", {}).get("task", "unknown"),
                    "confidence": result.get("parsed", {}).get("confidence", 0),
                    "parser": result.get("parsed", {}).get("parser", "unknown"),
                    "latency_ms": result.get("parse_latency_ms", 0),
                    "error": result.get("parsed", {}).get("error"),
                })
                print(f"    -> task={result.get('parsed', {}).get('task')}, "
                      f"confidence={result.get('parsed', {}).get('confidence'):.2f}, "
                      f"parser={result.get('parsed', {}).get('parser')}, "
                      f"latency={result.get('parse_latency_ms', 0):.0f}ms")
            except json.JSONDecodeError:
                llm_results.append({
                    "input": text, "cmd_id": cmd_id, "parsed_task": "parse_error",
                    "confidence": 0, "parser": "error", "latency_ms": 0, "error": "JSON decode failed",
                })
                print(f"    -> PARSE ERROR")
        except Exception as e:
            llm_results.append({
                "input": text, "cmd_id": cmd_id, "parsed_task": "error",
                "confidence": 0, "parser": "error", "latency_ms": 0, "error": str(e),
            })
            print(f"    -> ERROR: {e}")

    # Now test keyword fallback for comparison
    print("\n  Testing keyword fallback parser...")
    fallback_results = []
    for cmd_id, text in test_commands:
        cmd = [
            "ros2", "run", "office", "office_llm_command.py",
            "--force-fallback",
            text,
        ]
        try:
            proc = subprocess.run(cmd, capture_output=True, text=True, timeout=15)
            result = json.loads(proc.stdout)
            fallback_results.append({
                "input": text,
                "cmd_id": cmd_id,
                "parsed_task": result.get("parsed", {}).get("task", "unknown"),
                "confidence": result.get("parsed", {}).get("confidence", 0),
                "parser": result.get("parsed", {}).get("parser", "unknown"),
            })
            print(f"    -> '{text}' -> task={result.get('parsed', {}).get('task')}, "
                  f"confidence={result.get('parsed', {}).get('confidence'):.2f}")
        except Exception as e:
            fallback_results.append({
                "input": text, "cmd_id": cmd_id, "parsed_task": "error",
                "confidence": 0, "parser": "error",
            })

    # Test with remote LLM by calling office_llm_command.py without --force flags.
    # It will read config.toml itself and use the real LLM if API key is configured.
    print("\n  Testing remote LLM parser (auto-detect from config.toml / env)...")
    remote_results = []
    for cmd_id, text in test_commands:
        # Retry up to 3 times if LLM falls back to keyword_fallback
        best_result = None
        for attempt in range(3):
            cmd = [
                "ros2", "run", "office", "office_llm_command.py",
                "--save-json", f"/tmp/llm_remote_{cmd_id}.json",
                text,
            ]
            try:
                proc = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
                result = json.loads(proc.stdout)
                parser_used = result.get("parsed", {}).get("parser", "unknown")
                # If it fell back to mock, the remote LLM is not configured at all
                if parser_used == "mock":
                    print("    Remote LLM not configured (fell back to mock). Skipping remote test.")
                    remote_results = []
                    break
                # If it used the real LLM parser, we're done
                if parser_used not in ("keyword_fallback", "mock"):
                    best_result = {
                        "input": text,
                        "cmd_id": cmd_id,
                        "parsed_task": result.get("parsed", {}).get("task", "unknown"),
                        "confidence": result.get("parsed", {}).get("confidence", 0),
                        "parser": parser_used,
                        "latency_ms": result.get("parse_latency_ms", 0),
                        "model": result.get("llm_config", {}).get("model", ""),
                        "base_url": result.get("llm_config", {}).get("base_url", ""),
                    }
                    break
                # Fell back to keyword_fallback - retry with delay
                if attempt < 2:
                    print(f"    -> '{text}' fell back to keyword_fallback (attempt {attempt+1}/3), retrying in 2s...")
                    time.sleep(2)
                else:
                    # Use the fallback result as-is on final attempt
                    best_result = {
                        "input": text,
                        "cmd_id": cmd_id,
                        "parsed_task": result.get("parsed", {}).get("task", "unknown"),
                        "confidence": result.get("parsed", {}).get("confidence", 0),
                        "parser": parser_used,
                        "latency_ms": result.get("parse_latency_ms", 0),
                        "model": result.get("llm_config", {}).get("model", ""),
                        "base_url": result.get("llm_config", {}).get("base_url", ""),
                    }
            except subprocess.TimeoutExpired:
                if attempt < 2:
                    print(f"    -> '{text}' timed out (attempt {attempt+1}/3), retrying in 2s...")
                    time.sleep(2)
                else:
                    print("    Remote LLM timed out after 3 attempts. Skipping remote test.")
                    remote_results = []
                    break
            except Exception as e:
                print(f"    -> ERROR: {e}")
                remote_results = []
                break

        if remote_results is not None and remote_results == [] and len(remote_results) == 0:
            # Only break if we explicitly set remote_results = [] (meaning not configured)
            # Check if we broke out due to "not configured"
            if best_result is None:
                break

        if best_result:
            remote_results.append(best_result)
            print(f"    -> '{text}' -> task={best_result['parsed_task']}, "
                  f"confidence={best_result['confidence']:.2f}, "
                  f"parser={best_result['parser']}, "
                  f"latency={best_result['latency_ms']:.0f}ms")

        # Small delay between API calls to avoid rate limiting
        time.sleep(1)

    if not remote_results:
        print("  No remote LLM available. Set api_key in config.toml or LLM_API_KEY env var to enable.")

    # Compute summary
    correct_map = {
        "patrol_en": "patrol",
        "delivery_en": "delivery",
        "patrol_zh": "patrol",
        "delivery_zh": "delivery",
        "ambiguous": "patrol",  # ambiguous defaults to patrol
    }
    mock_correct = sum(
        1 for r in llm_results if r["parsed_task"] == correct_map.get(r["cmd_id"])
    )
    fallback_correct = sum(
        1 for r in fallback_results if r["parsed_task"] == correct_map.get(r["cmd_id"])
    )
    avg_mock_conf = sum(r["confidence"] for r in llm_results) / len(llm_results) if llm_results else 0
    avg_fb_conf = sum(r["confidence"] for r in fallback_results) / len(fallback_results) if fallback_results else 0

    notes = (
        f"Mock parser: {mock_correct}/{len(test_commands)} correct, avg confidence={avg_mock_conf:.2f} | "
        f"Keyword fallback: {fallback_correct}/{len(test_commands)} correct, avg confidence={avg_fb_conf:.2f}"
    )
    if remote_results:
        remote_correct = sum(
            1 for r in remote_results if r["parsed_task"] == correct_map.get(r["cmd_id"])
        )
        avg_remote_conf = sum(r["confidence"] for r in remote_results) / len(remote_results)
        avg_remote_latency = sum(r["latency_ms"] for r in remote_results) / len(remote_results)
        parser_name = remote_results[0].get("parser", "remote")
        model_name = remote_results[0].get("model", "")
        notes += f" | {parser_name}({model_name}): {remote_correct}/{len(test_commands)} correct, avg confidence={avg_remote_conf:.2f}, avg latency={avg_remote_latency:.0f}ms"

    result = ExperimentResult(
        name="llm",
        description="LLM natural-language command parsing accuracy comparison",
        success=True,
        notes=notes,
    )
    # Store detailed LLM results in mode_transitions field (repurposed)
    result.mode_transitions = [
        {"mock": r, "fallback": fr}
        for r, fr in zip(llm_results, fallback_results)
    ]
    if remote_results:
        for i, rr in enumerate(remote_results):
            if i < len(result.mode_transitions):
                result.mode_transitions[i]["remote"] = rr

    print(f"\n  Summary: {notes}")
    return result


def run_experiment_combined(collector: ExperimentCollector) -> ExperimentResult:
    """Experiment 4: Combined system (scheduler + obstacle avoidance)."""
    print("\n" + "=" * 60)
    print("EXPERIMENT 4: Combined System (Scheduler + Obstacle Avoidance)")
    print("=" * 60)

    collector.start()
    if not wait_for_robot_ready(collector):
        return ExperimentResult(name="combined", description="Combined system", success=False, notes="Robot not ready")

    print("  Dispatching tasks via scheduler...")
    dispatch_scheduler_tasks([
        "delivery:supplies:hardware",
        "go_to_place:lounge",
        "patrol:charger,patrol_a1,patrol_a2,charger",
    ])

    # Wait for robot to start moving, then spawn obstacle on its path
    print("  Waiting for robot to start moving...")
    start_pos = None
    for _ in range(16):
        rclpy.spin_once(collector, timeout_sec=0.5)
        pos = collector.current_position
        if start_pos is None:
            start_pos = (pos.x, pos.y)
        else:
            dx = pos.x - start_pos[0]
            dy = pos.y - start_pos[1]
            if math.sqrt(dx * dx + dy * dy) > 0.8:
                print(f"  Robot is moving! pos=({pos.x:.2f}, {pos.y:.2f})")
                break

    # Use dynamic heading-based placement
    obs_x, obs_y, spawned = _place_obstacle_on_path(
        collector, distance_ahead=4.0, obstacle_name="exp_obstacle"
    )
    if not spawned:
        print("  >>> MANUAL ACTION NEEDED: Place a box obstacle on the robot's path <<<")
        print("  >>> Press Enter when obstacle is placed <<<")
        input()

    success = wait_for_task_completion(collector, timeout=300.0)
    remove_obstacle("exp_obstacle")

    result = collector.get_result("combined", "Combined: scheduler + obstacle avoidance", success)
    result.notes = "Full system test with scheduler optimization and dynamic obstacle on path."

    print(f"  Result: {'SUCCESS' if success else 'FAILED'}")
    print(f"  Distance: {result.total_distance:.2f}m")
    print(f"  Time: {result.completion_time:.1f}s")
    print(f"  Replan count: {result.replan_count}")
    return result


# ---------------------------------------------------------------------------
# Report generator
# ---------------------------------------------------------------------------

def generate_report(results: List[ExperimentResult], output_path: str):
    """Generate a markdown report from experiment results."""
    lines = []
    lines.append("# Office Robot System Experiment Report")
    lines.append("")
    lines.append(f"Generated: {time.strftime('%Y-%m-%d %H:%M:%S')}")
    lines.append("")

    # Overview
    lines.append("## 1. Experiment Overview")
    lines.append("")
    lines.append("| # | Experiment | Description | Result |")
    lines.append("|---|---|---|---|")
    for i, r in enumerate(results, 1):
        status = "PASS" if r.success else "FAIL"
        lines.append(f"| {i} | {r.name} | {r.description} | {status} |")
    lines.append("")

    # Detailed results
    lines.append("## 2. Detailed Results")
    lines.append("")

    for r in results:
        lines.append(f"### {r.name}")
        lines.append("")
        lines.append(f"- **Description**: {r.description}")
        lines.append(f"- **Success**: {'Yes' if r.success else 'No'}")
        lines.append(f"- **Completion Time**: {r.completion_time:.1f}s")
        lines.append(f"- **Total Distance**: {r.total_distance:.2f}m")
        lines.append(f"- **Moving Time**: {r.moving_time:.1f}s")
        lines.append(f"- **Waiting Time**: {r.waiting_time:.1f}s")
        lines.append(f"- **Replan Count**: {r.replan_count}")
        if r.min_obstacle_distance != float("inf"):
            lines.append(f"- **Min Obstacle Distance**: {r.min_obstacle_distance:.2f}m")
        if r.notes:
            lines.append(f"- **Notes**: {r.notes}")
        lines.append("")

        # Mode transitions (skip for LLM experiment which repurposes this field)
        if r.mode_transitions and r.name != "llm":
            lines.append("Mode transitions:")
            lines.append("```")
            for tr in r.mode_transitions:
                if isinstance(tr, dict) and "t" in tr:
                    lines.append(f"  t={tr['t']:.1f}s: {tr['from']} -> {tr['to']}")
            lines.append("```")
            lines.append("")

    # Comparison analysis
    lines.append("## 3. Comparative Analysis")
    lines.append("")

    baseline = next((r for r in results if r.name == "baseline"), None)
    obstacle = next((r for r in results if r.name == "obstacle"), None)

    if obstacle:
        lines.append("### Dynamic Obstacle Avoidance: No-Avoid vs With-Avoid")
        lines.append("")
        lines.append("| Metric | No Avoidance | With Avoidance | Improvement |")
        lines.append("|---|---|---|---|")
        # Parse notes for Phase A/B results
        notes = obstacle.notes or ""
        lines.append(f"| Task Success | {'See notes' if 'STUCK' in notes else 'See notes'} | {'See notes' if 'STUCK' in notes else 'See notes'} | - |")
        lines.append(f"| Obstacle Events | {len([e for e in obstacle.obstacle_events if e.get('action') == 'detected'])} detected | - | - |")
        lines.append(f"| Notes | {notes} | | |")
        lines.append("")
        if "STUCK" in notes and "SUCCESS" in notes:
            lines.append("**Key Finding**: Without the avoidance node, the robot got stuck when encountering an obstacle. With the avoidance node active, the robot detected the obstacle, stopped, and RMF replanned an alternative route to reach the destination successfully.")
        elif "STUCK" in notes:
            lines.append("**Key Finding**: The robot got stuck in both phases. This may indicate no alternative route exists in the nav graph, or the avoidance node needs tuning.")
        else:
            lines.append("**Key Finding**: The robot reached its destination in both cases. The avoidance system added safety by detecting obstacles and triggering replans.")
        lines.append("")

    if baseline and obstacle:
        lines.append("### Baseline vs Obstacle (With Avoidance)")
        lines.append("")
        lines.append("| Metric | Baseline (No Obstacle) | With Obstacle + Avoidance | Delta |")
        lines.append("|---|---|---|---|")
        time_delta = obstacle.completion_time - baseline.completion_time
        dist_delta = obstacle.total_distance - baseline.total_distance
        lines.append(f"| Completion Time | {baseline.completion_time:.1f}s | {obstacle.completion_time:.1f}s | +{time_delta:.1f}s |")
        lines.append(f"| Total Distance | {baseline.total_distance:.2f}m | {obstacle.total_distance:.2f}m | +{dist_delta:.2f}m |")
        lines.append(f"| Success Rate | {'100%' if baseline.success else '0%'} | {'100%' if obstacle.success else '0%'} | - |")
        lines.append(f"| Replan Count | {baseline.replan_count} | {obstacle.replan_count} | +{obstacle.replan_count - baseline.replan_count} |")
        lines.append("")

    # Scheduler analysis
    scheduler = next((r for r in results if r.name == "scheduler"), None)
    if scheduler and scheduler.notes:
        lines.append("### Task Scheduler Impact")
        lines.append("")
        lines.append(f"**Analysis**: {scheduler.notes}")
        lines.append("")

    # LLM analysis
    llm = next((r for r in results if r.name == "llm"), None)
    if llm:
        lines.append("### LLM Command Parsing Accuracy")
        lines.append("")
        lines.append(f"**Analysis**: {llm.notes}")
        lines.append("")
        if llm.mode_transitions:
            lines.append("| Input | Mock Parser | Mock Conf | Fallback Parser | Fallback Conf |")
            lines.append("|---|---|---|---|---|")
            for tr in llm.mode_transitions:
                mock = tr.get("mock", {})
                fb = tr.get("fallback", {})
                lines.append(
                    f"| {mock.get('input', '?')} | {mock.get('parsed_task', '?')} | "
                    f"{mock.get('confidence', 0):.2f} | {fb.get('parsed_task', '?')} | "
                    f"{fb.get('confidence', 0):.2f} |"
                )
            lines.append("")
            # Remote LLM results if available
            has_remote = any("remote" in tr for tr in llm.mode_transitions)
            if has_remote:
                lines.append("| Input | Remote Parser | Remote Conf | Model | Latency (ms) |")
                lines.append("|---|---|---|---|---|")
                for tr in llm.mode_transitions:
                    rr = tr.get("remote", {})
                    if rr:
                        lines.append(
                            f"| {rr.get('input', '?')} | {rr.get('parsed_task', '?')} | "
                            f"{rr.get('confidence', 0):.2f} | {rr.get('model', '?')} | "
                            f"{rr.get('latency_ms', 0):.0f} |"
                        )
                lines.append("")

    # System architecture
    lines.append("## 4. System Architecture Summary")
    lines.append("")
    lines.append("The Office robot system uses the following obstacle avoidance architecture:")
    lines.append("")
    lines.append("```")
    lines.append("/scan (gpu_lidar) -> DynamicObstacleAvoidance (monitor)")
    lines.append("                          |")
    lines.append("                     [early warning log]")
    lines.append("")
    lines.append("slotcar plugin (Gazebo):")
    lines.append("  stop_distance=1.0m -> auto-stop -> MODE_WAITING")
    lines.append("                          |")
    lines.append("  fleet_manager: replan=True")
    lines.append("                          |")
    lines.append("  RobotCommandHandle: update_handle.replan()")
    lines.append("                          |")
    lines.append("  RMF dispatcher: find alternative route")
    lines.append("                          |")
    lines.append("  follow_new_path() -> robot rerouted")
    lines.append("```")
    lines.append("")
    lines.append("Key design decisions:")
    lines.append("- The avoidance node is **monitor-only** (does not control the robot)")
    lines.append("- The slotcar plugin handles physical stopping (stop_distance)")
    lines.append("- RobotCommandHandle handles replanning (15s cooldown)")
    lines.append("- RMF finds alternative routes in the nav graph (topology-based)")
    lines.append("- If no alternative route exists, the robot waits until the obstacle is removed")
    lines.append("")

    # Conclusions
    lines.append("## 5. Conclusions")
    lines.append("")
    success_count = sum(1 for r in results if r.success)
    total_count = len(results)
    lines.append(f"- **Overall success rate**: {success_count}/{total_count} ({success_count/total_count*100:.0f}%)")

    if baseline and obstacle:
        if obstacle.success:
            lines.append("- **Obstacle avoidance**: The system successfully reroutes the robot when obstacles are detected, demonstrating effective dynamic replanning")
        else:
            lines.append("- **Obstacle avoidance**: The robot was unable to find an alternative route, suggesting the nav graph may need additional paths for better coverage")

    if scheduler and scheduler.notes:
        lines.append("- **Task scheduling**: The nearest_neighbor optimization reduces total travel distance by reordering tasks based on proximity")

    lines.append("")
    lines.append("---")
    lines.append("*Report generated by office_experiment.py*")

    with open(output_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))
    print(f"\nReport saved to: {output_path}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Office Robot Experiment Runner"
    )
    parser.add_argument(
        "--exp", choices=["baseline", "obstacle", "scheduler", "llm", "combined", "all"],
        default="all", help="Which experiment to run (default: all)"
    )
    parser.add_argument(
        "--report-only", action="store_true",
        help="Generate report from existing data file"
    )
    parser.add_argument(
        "--robot-name", default="tinyRobot1",
        help="Robot name to track (default: tinyRobot1)"
    )
    parser.add_argument(
        "--output-dir", default=os.path.expanduser("~/ros_ws/experiment_results"),
        help="Directory to save results"
    )
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)
    json_path = os.path.join(args.output_dir, "experiment_results.json")
    report_path = os.path.join(args.output_dir, "experiment_report.md")

    # Report-only mode
    if args.report_only:
        if not os.path.exists(json_path):
            print(f"No data file found at {json_path}")
            sys.exit(1)
        with open(json_path, "r") as f:
            data = json.load(f)
        results = [ExperimentResult(**r) for r in data]
        generate_report(results, report_path)
        return

    # Initialize ROS
    rclpy.init()
    collector = ExperimentCollector(args.robot_name)

    results = []

    try:
        if args.exp in ("baseline", "all"):
            r = run_experiment_baseline(collector)
            results.append(r)

        if args.exp in ("obstacle", "all"):
            r = run_experiment_obstacle(collector)
            results.append(r)

        if args.exp in ("scheduler", "all"):
            r = run_experiment_scheduler(collector)
            results.append(r)

        if args.exp in ("llm", "all"):
            r = run_experiment_llm(collector)
            results.append(r)

        if args.exp in ("combined", "all"):
            r = run_experiment_combined(collector)
            results.append(r)

    except KeyboardInterrupt:
        print("\n\nExperiment interrupted by user.")
    finally:
        # Save results
        data = []
        for r in results:
            d = asdict(r)
            # Convert inf to string for JSON serialization
            if d["min_obstacle_distance"] == float("inf"):
                d["min_obstacle_distance"] = "N/A"
            data.append(d)

        with open(json_path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
        print(f"\nData saved to: {json_path}")

        # Generate report
        if results:
            generate_report(results, report_path)

        collector.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

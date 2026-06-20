#!/usr/bin/env python3
"""Dynamic obstacle avoidance node for Office robots.

Subscribes to /scan and robot_state topics, detects dynamic obstacles
within a configurable range, and monitors the robot's avoidance behavior.

Architecture:
  The slotcar plugin (Gazebo) has built-in obstacle detection:
    - stop_distance: stops the robot when an obstacle is within this range
    - stop_radius: lateral detection radius
  When stopped, the slotcar publishes MODE_WAITING via robot_state.
  The fleet_manager detects MODE_WAITING and sets replan=True.
  RobotCommandHandle polls requires_replan() and calls replan().
  RMF re-plans the route; if an alternative path exists in the nav
  graph, the robot will be rerouted.

  This node provides:
    1. EARLY WARNING via /scan (detects obstacles before slotcar stops)
    2. MONITORING of robot state (MODE_WAITING detection)
    3. LOGGING of obstacle events for diagnostics

  IMPORTANT: This node does NOT control the robot directly. It does NOT
  call stop_robot (which clears the navigation path and causes the robot
  to lose its destination). The slotcar's built-in mechanism handles
  stopping, and the RobotCommandHandle's replan mechanism handles
  rerouting.

Configuration is read from config.toml [obstacle_avoidance] section,
with sensible defaults if the file or section is missing.
"""

import argparse
import math
import os
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import LaserScan
from rmf_fleet_msgs.msg import RobotState, RobotMode

# ---------------------------------------------------------------------------
# Config helpers (shared with office_llm_command.py)
# ---------------------------------------------------------------------------

_PROJECT_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(__file__), os.pardir, os.pardir)
)
_CONFIG_PATH = os.path.join(_PROJECT_ROOT, "config.toml")


def _load_toml(path):
    data = {}
    current_section = data
    with open(path, "r", encoding="utf-8") as f:
        for raw_line in f:
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            if line.startswith("["):
                section_name = line.strip("[]").strip()
                parts = section_name.split(".")
                target = data
                for part in parts:
                    target = target.setdefault(part, {})
                current_section = target
                continue
            if "=" not in line:
                continue
            key, _, value = line.partition("=")
            key = key.strip()
            value = value.strip()
            if value.startswith('"'):
                end_quote = value.find('"', 1)
                if end_quote != -1:
                    value = value[1:end_quote]
                else:
                    value = value.strip('"')
            else:
                comment_idx = value.find("#")
                if comment_idx != -1:
                    value = value[:comment_idx].strip()
                if value == "true":
                    value = True
                elif value == "false":
                    value = False
                else:
                    try:
                        value = int(value)
                    except ValueError:
                        try:
                            value = float(value)
                        except ValueError:
                            pass
            current_section[key] = value
    return data


def _get_obstacle_config():
    defaults = {
        "enabled": True,
        "scan_topic": "/scan",
        "robot_state_topic": "robot_state",
        "robot_name": "tinyRobot1",
        "fleet_name": "tinyRobot",
        "obstacle_range_threshold": 1.5,
        "confirm_count": 3,
        "slowdown_factor": 0.3,
        "replan_cooldown_sec": 10.0,
        "fleet_manager_url": "http://127.0.0.1:22011",
    }
    if os.path.isfile(_CONFIG_PATH):
        try:
            cfg = _load_toml(_CONFIG_PATH).get("obstacle_avoidance", {})
            defaults.update({k: v for k, v in cfg.items() if k in defaults})
        except Exception:
            pass
    return defaults


# ---------------------------------------------------------------------------
# Dynamic Obstacle Avoidance Node
# ---------------------------------------------------------------------------


class DynamicObstacleAvoidance(Node):
    """Monitors /scan for dynamic obstacles and tracks the robot's state.

    This node does NOT directly control the robot. The slotcar plugin
    handles stopping (via stop_distance), and RobotCommandHandle handles
    replanning (via update_handle.replan()). This node provides early
    warning, monitoring, and diagnostic logging.
    """

    def __init__(self, cfg: dict):
        super().__init__("dynamic_obstacle_avoidance")
        self.cfg = cfg
        self.robot_name = cfg["robot_name"]
        self.fleet_name = cfg["fleet_name"]
        self.threshold = cfg["obstacle_range_threshold"]
        self.confirm_required = cfg["confirm_count"]

        # State
        self._robot_pose = None  # type: Optional[RobotState]
        self._confirm_counter = 0
        self._is_avoiding = False
        self._robot_mode = RobotMode.MODE_IDLE
        self._stuck_since = None  # type: Optional[float]

        # Subscribers
        scan_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            LaserScan, cfg["scan_topic"], self._scan_cb, scan_qos
        )
        state_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            RobotState, cfg["robot_state_topic"], self._robot_state_cb, state_qos
        )

        # Periodic status timer: log position + scan status every 2s
        self._last_scan_min = float("inf")
        self._scan_count = 0
        self.create_timer(2.0, self._status_timer_cb)

        self.get_logger().info(
            f"Dynamic obstacle avoidance started: robot={self.robot_name}, "
            f"threshold={self.threshold}m, confirm={self.confirm_required}"
        )
        self.get_logger().info(
            "Avoidance mode: MONITOR ONLY. The slotcar plugin handles "
            "stopping (stop_distance), RobotCommandHandle handles replanning."
        )

    # ----- callbacks -----

    def _status_timer_cb(self):
        """Periodic diagnostic: log robot position, scan status, and mode."""
        mode_str = self._mode_to_str(self._robot_mode)
        if self._robot_pose is not None:
            loc = self._robot_pose.location
            self.get_logger().info(
                f"[STATUS] {self.robot_name} pos=({loc.x:.2f}, {loc.y:.2f}, "
                f"yaw={loc.yaw:.2f}) | scans={self._scan_count} | "
                f"fwd_min={self._last_scan_min:.2f}m | "
                f"avoiding={self._is_avoiding} | mode={mode_str}"
            )
        else:
            self.get_logger().warn(
                f"[STATUS] {self.robot_name} NO robot_state received yet | "
                f"scans={self._scan_count} | fwd_min={self._last_scan_min:.2f}m"
            )

        # Detect if robot is stuck in MODE_WAITING for too long
        if self._robot_mode == RobotMode.MODE_WAITING:
            if self._stuck_since is None:
                self._stuck_since = time.time()
            elif time.time() - self._stuck_since > 30.0:
                self.get_logger().warn(
                    f"[STUCK] {self.robot_name} has been in MODE_WAITING for "
                    f"{time.time() - self._stuck_since:.0f}s. "
                    f"The obstacle may be permanent and no alternative route "
                    f"exists in the nav graph. Remove the obstacle or wait "
                    f"for RMF to find an alternative path."
                )
        else:
            self._stuck_since = None

    def _robot_state_cb(self, msg: RobotState):
        if msg.name == self.robot_name:
            old_mode = self._robot_mode
            self._robot_pose = msg
            self._robot_mode = msg.mode.mode

            # Log mode transitions
            if old_mode != self._robot_mode:
                self.get_logger().info(
                    f"[MODE] {self.robot_name} transition: "
                    f"{self._mode_to_str(old_mode)} -> "
                    f"{self._mode_to_str(self._robot_mode)}"
                )

    def _scan_cb(self, scan: LaserScan):
        if not self.cfg["enabled"]:
            return

        # Find the closest obstacle in the forward sector (+-60 deg)
        angle_min = scan.angle_min
        angle_increment = scan.angle_increment
        forward_half = math.radians(60)
        min_range = float("inf")

        for i, r in enumerate(scan.ranges):
            if math.isinf(r) or math.isnan(r) or r < scan.range_min:
                continue
            angle = angle_min + i * angle_increment
            # Normalize to [-pi, pi]
            angle = (angle + math.pi) % (2 * math.pi) - math.pi
            if abs(angle) <= forward_half:
                if r < min_range:
                    min_range = r

        self._last_scan_min = min_range
        self._scan_count += 1

        obstacle_close = min_range <= self.threshold

        if obstacle_close:
            self._confirm_counter += 1
        else:
            self._confirm_counter = max(0, self._confirm_counter - 1)

        if self._confirm_counter >= self.confirm_required and not self._is_avoiding:
            self._trigger_avoidance(min_range)
        elif not obstacle_close and self._is_avoiding:
            self._clear_avoidance()

    # ----- avoidance detection -----

    def _trigger_avoidance(self, closest_range: float):
        self._is_avoiding = True
        self.get_logger().warn(
            f"[OBSTACLE] Dynamic obstacle detected at {closest_range:.2f}m! "
            f"The slotcar will stop at its stop_distance, then RMF will "
            f"attempt to replan. If an alternative route exists, the robot "
            f"will be rerouted."
        )

    def _clear_avoidance(self):
        self._is_avoiding = False
        self._confirm_counter = 0
        self.get_logger().info(
            "[CLEAR] Obstacle cleared. The slotcar should resume "
            "automatically or RMF will replan."
        )

    # ----- helpers -----

    @staticmethod
    def _mode_to_str(mode: int) -> str:
        mode_map = {
            RobotMode.MODE_IDLE: "IDLE",
            RobotMode.MODE_CHARGING: "CHARGING",
            RobotMode.MODE_MOVING: "MOVING",
            RobotMode.MODE_PAUSED: "PAUSED",
            RobotMode.MODE_WAITING: "WAITING",
            RobotMode.MODE_EMERGENCY: "EMERGENCY",
            RobotMode.MODE_DOCKING: "DOCKING",
            RobotMode.MODE_ADAPTER_ERROR: "ADAPTER_ERROR",
        }
        return mode_map.get(mode, f"UNKNOWN({mode})")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main():
    parser = argparse.ArgumentParser(
        description="Dynamic obstacle avoidance node for Office robots."
    )
    parser.add_argument("--robot-name", default=None, help="Override robot name.")
    parser.add_argument("--threshold", type=float, default=None, help="Override obstacle range threshold (m).")
    args = parser.parse_args()

    cfg = _get_obstacle_config()
    if args.robot_name:
        cfg["robot_name"] = args.robot_name
    if args.threshold is not None:
        cfg["obstacle_range_threshold"] = args.threshold

    rclpy.init()
    node = DynamicObstacleAvoidance(cfg)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

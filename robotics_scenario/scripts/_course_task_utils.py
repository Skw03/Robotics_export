#!/usr/bin/env python3

import json
import math
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from robotics_interfaces.action import Delivery
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


SEMANTIC_POSES: Dict[str, Dict[str, Dict[str, float]]] = {
    "warehouse": {
        "staging_area": {"x": -3.07, "y": 3.58, "yaw": 0.0},
        "demo_checkpoint": {"x": -3.40, "y": 2.95, "yaw": 0.0},
        "inbound": {"x": 0.45, "y": 3.00, "yaw": -1.57},
        "shelf_west": {"x": 3.30, "y": 2.10, "yaw": 3.14},
        "shelf_center": {"x": 3.30, "y": 1.00, "yaw": 1.57},
        "shelf_east": {"x": 4.73, "y": -1.24, "yaw": 0.0},
        "outbound": {"x": -0.28, "y": -9.48, "yaw": 0.0},
        "charging_dock": {"x": -1.40, "y": 2.85, "yaw": 0.0},
    },
    "office": {
        "charger": {"x": 55.07, "y": -58.48, "yaw": 0.0},
        "supplies": {"x": 59.68, "y": -31.66, "yaw": 1.57},
        "pantry": {"x": 69.81, "y": -93.92, "yaw": 0.0},
        "lounge": {"x": 85.86, "y": -112.07, "yaw": 0.0},
        "hardware": {"x": 66.93, "y": -121.32, "yaw": 3.14},
        "coe": {"x": 47.49, "y": -28.80, "yaw": 0.0},
        "patrol_a1": {"x": 46.49, "y": -59.56, "yaw": 1.57},
        "patrol_a2": {"x": 81.95, "y": -102.73, "yaw": 0.0},
        "patrol_d1": {"x": 61.58, "y": -97.15, "yaw": 3.14},
        "patrol_c": {"x": 42.35, "y": -117.10, "yaw": 3.14},
        "patrol_b": {"x": 20.76, "y": -56.02, "yaw": 1.57},
        "patrol_d2": {"x": 68.54, "y": -52.11, "yaw": -1.57},
        "backup_charger": {"x": 78.01, "y": -113.70, "yaw": 3.14},
    },
}

TASK_PRESETS = {
    "warehouse": {
        "delivery": {
            "task_type": "rack_to_dropoff",
            "semantic_goal_id": "warehouse_aws_fulfillment",
            "route": [
                "staging_area",
                "inbound",
                "shelf_west",
                "shelf_center",
                "shelf_east",
                "outbound",
                "charging_dock",
            ],
        },
        "patrol": {
            "task_type": "patrol_loop",
            "semantic_goal_id": "warehouse_aws_patrol",
            "route": [
                "charging_dock",
                "staging_area",
                "inbound",
                "shelf_center",
                "shelf_east",
                "outbound",
                "charging_dock",
            ],
        },
        "demo": {
            "task_type": "stage2_demo",
            "semantic_goal_id": "warehouse_stage2_demo_loop",
            "route": [
                "staging_area",
                "demo_checkpoint",
                "staging_area",
            ],
        },
    },
    "office": {
        "delivery": {
            "task_type": "mail_delivery",
            "semantic_goal_id": "office_mail_delivery",
            "route": [
                "charger",
                "supplies",
                "pantry",
                "lounge",
                "hardware",
                "coe",
                "charger",
            ],
        },
        "patrol": {
            "task_type": "patrol_loop",
            "semantic_goal_id": "office_patrol_loop",
            "route": [
                "charger",
                "patrol_a1",
                "patrol_a2",
                "patrol_d1",
                "patrol_c",
                "patrol_b",
                "charger",
            ],
        },
        "demo": {
            "task_type": "stage2_demo",
            "semantic_goal_id": "office_stage2_demo_loop",
            "route": [
                "charger",
                "patrol_a1",
            ],
        },
    },
}

SCENE_ACTION_NAMES = {
    "warehouse": "warehouse_delivery_scenario",
    "office": "office_delivery_scenario",
}


@dataclass
class DispatchResult:
    accepted: bool
    status: str
    task_status: str
    elapsed_sec: float
    task_spec: Dict[str, object]


def yaw_to_quaternion(yaw: float):
    return math.sin(yaw * 0.5), math.cos(yaw * 0.5)


def build_pose(x: float, y: float, yaw: float) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0
    z, w = yaw_to_quaternion(float(yaw))
    pose.pose.orientation.z = z
    pose.pose.orientation.w = w
    return pose


def build_goal(scene: str, task_name: str) -> Delivery.Goal:
    scene_presets = TASK_PRESETS[scene]
    if task_name not in scene_presets:
        raise KeyError(f"Task '{task_name}' is not defined for scene '{scene}'")

    preset = scene_presets[task_name]
    route: List[str] = preset["route"]
    poses = SEMANTIC_POSES[scene]

    goal = Delivery.Goal()
    goal.scene_id = scene
    goal.task_type = preset["task_type"]
    goal.semantic_goal_id = preset["semantic_goal_id"]
    goal.start_floor = "1f"
    goal.target_floor = "1f"
    goal.return_floor = "1f"
    goal.semantic_route = route
    goal.start_pose = build_pose(**poses[route[0]])
    goal.end_pose = build_pose(**poses[route[-2]])
    goal.return_pose = build_pose(**poses[route[-1]])
    goal.behavior_tree = ""
    return goal


class TaskDispatchNode(Node):
    def __init__(self):
        super().__init__("course_task_dispatcher")
        self._collecting_metrics = False
        self._path_length_m = 0.0
        self._last_odom_xy: Optional[Tuple[float, float]] = None
        self._min_obstacle_dist_m = float("inf")
        self._near_collision_events = 0
        self._near_collision_active = False
        self._near_collision_threshold_m = 0.25
        self.create_subscription(Odometry, "/odom", self._on_odom, 20)
        self.create_subscription(LaserScan, "/scan", self._on_scan, 20)

    def _reset_metrics(self):
        self._path_length_m = 0.0
        self._last_odom_xy = None
        self._min_obstacle_dist_m = float("inf")
        self._near_collision_events = 0
        self._near_collision_active = False

    def _on_odom(self, msg: Odometry):
        if not self._collecting_metrics:
            return
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        if self._last_odom_xy is not None:
            dx = x - self._last_odom_xy[0]
            dy = y - self._last_odom_xy[1]
            self._path_length_m += math.hypot(dx, dy)
        self._last_odom_xy = (x, y)

    def _on_scan(self, msg: LaserScan):
        if not self._collecting_metrics:
            return
        finite_ranges = [r for r in msg.ranges if math.isfinite(r) and r > 0.0]
        if not finite_ranges:
            return
        min_range = min(finite_ranges)
        if min_range < self._min_obstacle_dist_m:
            self._min_obstacle_dist_m = min_range

        is_near = min_range <= self._near_collision_threshold_m
        if is_near and not self._near_collision_active:
            self._near_collision_events += 1
        self._near_collision_active = is_near

    def _start_metrics_collection(self, near_collision_threshold_m: float):
        self._near_collision_threshold_m = near_collision_threshold_m
        self._reset_metrics()
        self._collecting_metrics = True

    def _stop_metrics_collection(self) -> Dict[str, float]:
        self._collecting_metrics = False
        min_obstacle_dist_m = self._min_obstacle_dist_m
        if not math.isfinite(min_obstacle_dist_m):
            min_obstacle_dist_m = -1.0
        return {
            "path_length_m": round(self._path_length_m, 3),
            "min_obstacle_dist_m": round(min_obstacle_dist_m, 3),
            "near_collision_events": int(self._near_collision_events),
            "near_collision_threshold_m": round(self._near_collision_threshold_m, 3),
        }

    def dispatch(
        self,
        scene: str,
        task_name: str,
        timeout_sec: float = 20.0,
        result_timeout_sec: float = 180.0,
        collect_metrics: bool = True,
        near_collision_threshold_m: float = 0.25,
    ) -> DispatchResult:
        action_name = SCENE_ACTION_NAMES[scene]
        client = ActionClient(self, Delivery, action_name)
        self.get_logger().info(f"Waiting for action server '{action_name}'")
        if not client.wait_for_server(timeout_sec=timeout_sec):
            raise RuntimeError(f"Action server '{action_name}' is not available")

        goal = build_goal(scene, task_name)
        if collect_metrics:
            self._start_metrics_collection(near_collision_threshold_m)
        start_time = time.time()
        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=timeout_sec)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            metrics = self._stop_metrics_collection() if collect_metrics else {}
            return DispatchResult(False, "REJECTED", "REJECTED", time.time() - start_time, {
                "scene": scene,
                "task": task_name,
                "route": list(goal.semantic_route),
                "metrics": metrics,
            })

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=result_timeout_sec)
        if not result_future.done():
            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=5.0)
            metrics = self._stop_metrics_collection() if collect_metrics else {}
            return DispatchResult(
                True,
                "TIMEOUT",
                "TIMEOUT",
                time.time() - start_time,
                {
                    "scene": scene,
                    "task": task_name,
                    "route": list(goal.semantic_route),
                    "semantic_goal_id": goal.semantic_goal_id,
                    "metrics": metrics,
                },
            )
        wrapped_result = result_future.result()
        result = wrapped_result.result
        metrics = self._stop_metrics_collection() if collect_metrics else {}
        return DispatchResult(
            True,
            result.final_status,
            result.task_status,
            time.time() - start_time,
            {
                "scene": scene,
                "task": task_name,
                "route": list(goal.semantic_route),
                "semantic_goal_id": goal.semantic_goal_id,
                "metrics": metrics,
            },
        )


def dump_result(result: DispatchResult) -> str:
    payload = {
        "accepted": result.accepted,
        "status": result.status,
        "task_status": result.task_status,
        "elapsed_sec": round(result.elapsed_sec, 3),
        "task_spec": result.task_spec,
    }
    return json.dumps(payload, indent=2, ensure_ascii=False)

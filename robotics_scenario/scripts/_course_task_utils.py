#!/usr/bin/env python3

import json
import math
import time
from dataclasses import dataclass
from typing import Dict, List

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from robotics_interfaces.action import Delivery


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
                "patrol_a1",
            ],
        },
        "patrol": {
            "task_type": "patrol_loop",
            "semantic_goal_id": "office_patrol_loop",
            "route": [
                "charger",
                "patrol_a1",
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

    def dispatch(
        self,
        scene: str,
        task_name: str,
        timeout_sec: float = 20.0,
        result_timeout_sec: float = 180.0,
    ) -> DispatchResult:
        action_name = SCENE_ACTION_NAMES[scene]
        client = ActionClient(self, Delivery, action_name)
        self.get_logger().info(f"Waiting for action server '{action_name}'")
        if not client.wait_for_server(timeout_sec=timeout_sec):
            raise RuntimeError(f"Action server '{action_name}' is not available")

        goal = build_goal(scene, task_name)
        start_time = time.time()
        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=timeout_sec)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return DispatchResult(False, "REJECTED", "REJECTED", time.time() - start_time, {
                "scene": scene,
                "task": task_name,
                "route": list(goal.semantic_route),
            })

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=result_timeout_sec)
        if not result_future.done():
            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=5.0)
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
                },
            )
        wrapped_result = result_future.result()
        result = wrapped_result.result
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

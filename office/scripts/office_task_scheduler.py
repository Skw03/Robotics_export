#!/usr/bin/env python3
"""Office Task Scheduler with route optimization.

Accepts multiple tasks (delivery, patrol, go_to_place), analyzes their
routes using the Office topology map, and dispatches them in an optimal
order to minimize total travel distance.

Supports two scheduling strategies:
  - nearest_neighbor: Greedy nearest-neighbor heuristic (fast, good enough).
  - greedy_tsp: Greedy TSP approximation with 2-opt improvement.

Configuration is read from config.toml [scheduler] section.
"""

import argparse
import asyncio
import json
import math
import os
import sys
import time
import uuid
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

# ---------------------------------------------------------------------------
# Config helpers
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
            # Strip inline comments (outside of quoted strings)
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


def _get_scheduler_config():
    defaults = {
        "strategy": "nearest_neighbor",
        "robot_name": "tinyRobot1",
        "fleet_name": "tinyRobot",
        "default_charger": "tinyRobot1_charger",
    }
    if os.path.isfile(_CONFIG_PATH):
        try:
            cfg = _load_toml(_CONFIG_PATH).get("scheduler", {})
            defaults.update({k: v for k, v in cfg.items() if k in defaults})
        except Exception:
            pass
    return defaults


# ---------------------------------------------------------------------------
# Office topology map (from office_topology.yaml / office_semantic_goals.yaml)
# ---------------------------------------------------------------------------

OFFICE_LOCATIONS: Dict[str, Tuple[float, float]] = {
    "charger": (55.07, -58.48),
    "supplies": (59.68, -31.66),
    "pantry": (69.81, -93.92),
    "lounge": (85.86, -112.07),
    "hardware": (66.93, -121.32),
    "coe": (47.49, -28.80),
    "patrol_a1": (46.49, -59.56),
    "patrol_a2": (81.95, -102.73),
    "patrol_d1": (61.58, -97.15),
    "patrol_c": (42.35, -117.10),
    "patrol_b": (20.76, -56.02),
    "patrol_d2": (68.54, -52.11),
    "backup_charger": (78.01, -113.70),
    "tinyRobot1_charger": (55.07, -58.48),
    "tinyRobot2_charger": (55.07, -58.48),
    "trash_room": (55.07, -58.48),
}

# Mapping from lowercase/alias names used in CLI to actual RMF nav graph waypoint names.
# Keys are the names used in OFFICE_LOCATIONS; values are the canonical names
# that RMF's navigation graph expects.
RMF_WAYPOINT_ALIASES: Dict[str, str] = {
    "charger": "tinyRobot1_charger",
    "patrol_a1": "patrol_A1",
    "patrol_a2": "patrol_A2",
    "patrol_b": "patrol_B",
    "patrol_c": "patrol_C",
    "patrol_d1": "patrol_D1",
    "patrol_d2": "patrol_D2",
    "hardware": "hardware_2",
    "trash_room": "trash_room",
    "backup_charger": "tinyRobot2_charger",
}


def resolve_waypoint_name(name: str) -> str:
    """Resolve a location name to the canonical RMF nav graph waypoint name.

    If the name has an alias mapping, return the canonical name.
    Otherwise return the name as-is (e.g. 'lounge', 'pantry', 'supplies'
    are already correct in the nav graph).
    """
    return RMF_WAYPOINT_ALIASES.get(name, name)


def euclidean_distance(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def location_distance(name1: str, name2: str) -> float:
    """Euclidean distance between two named locations."""
    p1 = OFFICE_LOCATIONS.get(name1)
    p2 = OFFICE_LOCATIONS.get(name2)
    if p1 is None or p2 is None:
        return float("inf")
    return euclidean_distance(p1, p2)


# ---------------------------------------------------------------------------
# Task definitions
# ---------------------------------------------------------------------------

@dataclass
class Task:
    """A single task to be scheduled."""
    task_id: str
    task_type: str  # delivery, patrol, go_to_place
    target_location: str  # primary destination
    route: List[str] = field(default_factory=list)  # ordered waypoints
    priority: int = 0  # higher = more urgent
    # For delivery tasks
    pickup: str = ""
    dropoff: str = ""
    pickup_handler: str = ""
    dropoff_handler: str = ""

    @property
    def start_location(self) -> str:
        if self.route:
            return self.route[0]
        return self.target_location

    @property
    def end_location(self) -> str:
        if self.route:
            return self.route[-1]
        return self.target_location


# ---------------------------------------------------------------------------
# Route optimization strategies
# ---------------------------------------------------------------------------

def nearest_neighbor_order(
    tasks: List[Task],
    start_location: str,
) -> Tuple[List[int], float]:
    """Greedy nearest-neighbor ordering starting from start_location.

    Returns (ordered_indices, total_distance).
    """
    n = len(tasks)
    if n == 0:
        return [], 0.0

    remaining = set(range(n))
    order = []
    current = start_location
    total_dist = 0.0

    while remaining:
        best_idx = None
        best_dist = float("inf")
        for idx in remaining:
            d = location_distance(current, tasks[idx].start_location)
            # Prefer higher priority (lower priority number = more urgent)
            if d < best_dist or (d == best_dist and tasks[idx].priority > (tasks[best_idx].priority if best_idx is not None else -1)):
                best_dist = d
                best_idx = idx
        if best_idx is None:
            break
        order.append(best_idx)
        remaining.remove(best_idx)
        total_dist += best_dist
        current = tasks[best_idx].end_location

    return order, total_dist


def greedy_tsp_order(
    tasks: List[Task],
    start_location: str,
) -> Tuple[List[int], float]:
    """Greedy TSP with 2-opt improvement.

    Returns (ordered_indices, total_distance).
    """
    order, total_dist = nearest_neighbor_order(tasks, start_location)
    if len(order) <= 2:
        return order, total_dist

    # 2-opt improvement
    def route_distance(indices: List[int]) -> float:
        dist = 0.0
        current = start_location
        for idx in indices:
            dist += location_distance(current, tasks[idx].start_location)
            current = tasks[idx].end_location
        return dist

    improved = True
    while improved:
        improved = False
        for i in range(len(order) - 1):
            for j in range(i + 1, len(order)):
                new_order = order[:i] + order[i:j + 1][::-1] + order[j + 1:]
                new_dist = route_distance(new_order)
                if new_dist < total_dist:
                    order = new_order
                    total_dist = new_dist
                    improved = True

    return order, total_dist


# ---------------------------------------------------------------------------
# Task Scheduler Node
# ---------------------------------------------------------------------------

class TaskSchedulerNode:
    """Schedules and dispatches multiple tasks in optimal order.

    Note: This class requires rclpy. Import it only when dispatching.
    """

    def __init__(self, cfg: dict):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
        from rmf_task_msgs.msg import ApiRequest, ApiResponse

        # We create a Node dynamically to avoid top-level rclpy import
        rclpy.init()
        self._node = Node("office_task_scheduler")
        self._rclpy = rclpy
        self.cfg = cfg
        self.robot_name = cfg["robot_name"]
        self.fleet_name = cfg["fleet_name"]
        self.default_charger = cfg["default_charger"]
        self.strategy = cfg["strategy"]

        transient_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._ApiRequest = ApiRequest
        self._ApiResponse = ApiResponse
        self._pub = self._node.create_publisher(ApiRequest, "task_api_requests", transient_qos)
        self._pending_futures: Dict[str, asyncio.Future] = {}

        self._node.create_subscription(
            ApiResponse, "task_api_responses", self._response_cb, 10
        )

    @property
    def get_logger(self):
        return self._node.get_logger

    def _response_cb(self, msg):
        if msg.request_id in self._pending_futures:
            self._pending_futures[msg.request_id].set_result(
                json.loads(msg.json_msg)
            )

    def _dispatch_task(self, task: Task, use_sim_time: bool) -> str:
        """Dispatch a single task via RMF ApiRequest. Returns request_id."""
        from rclpy.parameter import Parameter

        if use_sim_time:
            param = Parameter("use_sim_time", Parameter.Type.BOOL, True)
            self._node.set_parameters([param])

        msg = self._ApiRequest()
        payload = {}

        # Use dispatch_task_request (goes through RMF dispatcher for bidding)
        # instead of robot_task_request (direct assignment, often not supported).
        # This matches the behavior of dispatch_patrol/dispatch_delivery tools.
        payload["type"] = "dispatch_task_request"

        now = self._node.get_clock().now().to_msg()
        start_time = now.sec * 1000 + round(now.nanosec / 10 ** 6)

        request = {
            "unix_millis_request_time": start_time,
            "unix_millis_earliest_start_time": start_time,
            "requester": "office_task_scheduler",
        }

        if task.task_type == "delivery":
            request["category"] = "delivery"
            request["description"] = {
                "pickup": {
                    "place": resolve_waypoint_name(task.pickup),
                    "handler": task.pickup_handler,
                    "payload": [],
                },
                "dropoff": {
                    "place": resolve_waypoint_name(task.dropoff),
                    "handler": task.dropoff_handler,
                    "payload": [],
                },
            }
        elif task.task_type == "patrol":
            request["category"] = "compose"
            activities = []
            for place in task.route:
                activities.append({"category": "go_to_place", "description": resolve_waypoint_name(place)})
            request["description"] = {
                "category": "patrol",
                "phases": [
                    {
                        "activity": {
                            "category": "sequence",
                            "description": {"activities": activities},
                        }
                    }
                ],
            }
        elif task.task_type == "go_to_place":
            request["category"] = "compose"
            request["description"] = {
                "category": "go_to_place_sequence",
                "phases": [
                    {
                        "activity": {
                            "category": "sequence",
                            "description": {
                                "activities": [
                                    {"category": "go_to_place", "description": resolve_waypoint_name(task.target_location)}
                                ]
                            },
                        }
                    }
                ],
            }
        else:
            raise ValueError(f"Unsupported task type: {task.task_type}")

        payload["request"] = request
        msg.json_msg = json.dumps(payload)
        msg.request_id = f"sched_{task.task_type}_{uuid.uuid4()}"

        future = asyncio.Future()
        self._pending_futures[msg.request_id] = future

        self._node.get_logger().info(
            f"Dispatching task [{task.task_id}] type={task.task_type} "
            f"target={task.target_location} request_id={msg.request_id}"
        )
        self._node.get_logger().info(
            f"Payload: {json.dumps(payload, indent=2)}"
        )
        self._pub.publish(msg)
        return msg.request_id

    def schedule_and_dispatch(
        self,
        tasks: List[Task],
        start_location: str = "charger",
        use_sim_time: bool = False,
        dry_run: bool = False,
    ) -> dict:
        """Schedule tasks in optimal order and dispatch them sequentially.

        Returns a dict with scheduling results.
        """
        if not tasks:
            return {"error": "No tasks to schedule", "tasks": [], "total_distance": 0.0}

        # Optimize task order
        if self.strategy == "greedy_tsp":
            order, total_dist = greedy_tsp_order(tasks, start_location)
        else:
            order, total_dist = nearest_neighbor_order(tasks, start_location)

        ordered_tasks = [tasks[i] for i in order]

        result = {
            "strategy": self.strategy,
            "start_location": start_location,
            "total_estimated_distance": round(total_dist, 2),
            "original_order": [t.task_id for t in tasks],
            "scheduled_order": [t.task_id for t in ordered_tasks],
            "tasks": [],
            "dry_run": dry_run,
        }

        self._node.get_logger().info(
            f"Scheduled {len(ordered_tasks)} tasks with strategy={self.strategy}, "
            f"estimated distance={total_dist:.2f}"
        )

        if dry_run:
            for i, task in enumerate(ordered_tasks):
                result["tasks"].append({
                    "task_id": task.task_id,
                    "task_type": task.task_type,
                    "target_location": task.target_location,
                    "route": task.route,
                    "priority": task.priority,
                    "position": i + 1,
                })
            return result

        # Dispatch tasks sequentially
        for i, task in enumerate(ordered_tasks):
            task_result = {
                "task_id": task.task_id,
                "task_type": task.task_type,
                "target_location": task.target_location,
                "route": task.route,
                "priority": task.priority,
                "position": i + 1,
            }
            try:
                request_id = self._dispatch_task(task, use_sim_time)
                task_result["request_id"] = request_id
                task_result["status"] = "dispatched"
                self._node.get_logger().info(
                    f"[{i + 1}/{len(ordered_tasks)}] Dispatched {task.task_id} -> {request_id}"
                )
            except Exception as exc:
                task_result["status"] = "failed"
                task_result["error"] = str(exc)
                self._node.get_logger().error(f"Failed to dispatch {task.task_id}: {exc}")

            result["tasks"].append(task_result)

        return result


# ---------------------------------------------------------------------------
# Task parsing helpers
# ---------------------------------------------------------------------------

def parse_task_from_args(task_args: List[str]) -> Task:
    """Parse a task specification from command-line arguments.

    Format: type:target[:route] or delivery:pickup:dropoff
    Examples:
      patrol:charger,patrol_a1,patrol_a2,patrol_d1,charger
      delivery:supplies:hardware
      go_to_place:lounge
    """
    parts = task_args[0].split(":", 2)
    task_type = parts[0]

    if task_type == "delivery" and len(parts) >= 3:
        pickup = parts[1]
        dropoff = parts[2]
        route = [pickup, dropoff]
        return Task(
            task_id=f"delivery_{uuid.uuid4().hex[:8]}",
            task_type="delivery",
            target_location=dropoff,
            route=route,
            pickup=pickup,
            dropoff=dropoff,
            pickup_handler=pickup,
            dropoff_handler=dropoff,
        )
    elif task_type == "patrol" and len(parts) >= 2:
        route_str = parts[1]
        route = [r.strip() for r in route_str.split(",") if r.strip()]
        if not route:
            route = [parts[1]]
        return Task(
            task_id=f"patrol_{uuid.uuid4().hex[:8]}",
            task_type="patrol",
            target_location=route[0],
            route=route,
        )
    elif task_type == "go_to_place" and len(parts) >= 2:
        target = parts[1]
        return Task(
            task_id=f"goto_{uuid.uuid4().hex[:8]}",
            task_type="go_to_place",
            target_location=target,
            route=[target],
        )
    else:
        # Fallback: treat as go_to_place
        target = parts[1] if len(parts) >= 2 else parts[0]
        return Task(
            task_id=f"goto_{uuid.uuid4().hex[:8]}",
            task_type="go_to_place",
            target_location=target,
            route=[target],
        )


# ---------------------------------------------------------------------------
# Standalone scheduling (no ROS dependency)
# ---------------------------------------------------------------------------

def schedule_tasks_standalone(
    tasks: List[Task],
    start_location: str = "charger",
    strategy: str = "nearest_neighbor",
) -> dict:
    """Schedule tasks without ROS. Used for dry-run mode."""
    if not tasks:
        return {"error": "No tasks to schedule", "tasks": [], "total_distance": 0.0}

    if strategy == "greedy_tsp":
        order, total_dist = greedy_tsp_order(tasks, start_location)
    else:
        order, total_dist = nearest_neighbor_order(tasks, start_location)

    ordered_tasks = [tasks[i] for i in order]

    result = {
        "strategy": strategy,
        "start_location": start_location,
        "total_estimated_distance": round(total_dist, 2),
        "original_order": [t.task_id for t in tasks],
        "scheduled_order": [t.task_id for t in ordered_tasks],
        "tasks": [],
        "dry_run": True,
    }

    for i, task in enumerate(ordered_tasks):
        result["tasks"].append({
            "task_id": task.task_id,
            "task_type": task.task_type,
            "target_location": task.target_location,
            "route": task.route,
            "priority": task.priority,
            "position": i + 1,
        })

    return result


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Office Task Scheduler - optimize and dispatch multiple tasks."
    )
    parser.add_argument(
        "tasks", nargs="+",
        help=(
            "Task specifications. Format: type:target[:route]. "
            "Examples: patrol:charger,patrol_a1,patrol_a2,charger "
            "delivery:supplies:hardware "
            "go_to_place:lounge"
        ),
    )
    parser.add_argument("--strategy", choices=["nearest_neighbor", "greedy_tsp"], default=None,
                        help="Scheduling strategy (overrides config.toml).")
    parser.add_argument("--start", default="charger", help="Starting location for route calculation.")
    parser.add_argument("--robot", default=None, help="Override robot name.")
    parser.add_argument("--fleet", default=None, help="Override fleet name.")
    parser.add_argument("--use_sim_time", action="store_true", help="Use simulation time.")
    parser.add_argument("--dry-run", action="store_true", help="Only show scheduling plan, do not dispatch.")
    parser.add_argument("--save-json", default=None, help="Save scheduling results to JSON file.")
    args = parser.parse_args()

    cfg = _get_scheduler_config()
    if args.strategy:
        cfg["strategy"] = args.strategy
    if args.robot:
        cfg["robot_name"] = args.robot
    if args.fleet:
        cfg["fleet_name"] = args.fleet

    # Parse tasks
    tasks = []
    for task_arg in args.tasks:
        tasks.append(parse_task_from_args([task_arg]))

    print(f"Parsed {len(tasks)} tasks:")
    for t in tasks:
        print(f"  - {t.task_id}: {t.task_type} -> {t.target_location} (route: {t.route})")

    if args.dry_run:
        # No ROS needed for dry-run - use standalone scheduling logic
        result = schedule_tasks_standalone(
            tasks, start_location=args.start, strategy=cfg["strategy"]
        )
    else:
        node = TaskSchedulerNode(cfg)
        try:
            result = node.schedule_and_dispatch(
                tasks, start_location=args.start,
                use_sim_time=args.use_sim_time, dry_run=False,
            )
        finally:
            node._node.destroy_node()
            node._rclpy.shutdown()

    output = json.dumps(result, indent=2, ensure_ascii=False)
    print(output)

    if args.save_json:
        path = os.path.abspath(args.save_json)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            f.write(output + "\n")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

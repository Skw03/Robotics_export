#!/usr/bin/env python3
"""Generate Office-only path planning and avoidance comparison evidence.

The analysis is intentionally offline: it uses the Office RMF navigation graph
as the shared map input, then compares graph planners and lane-closure
replanning without modifying the running Office/RMF stack.
"""

from __future__ import annotations

import argparse
import csv
import heapq
import json
import math
import pathlib
import time
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Set, Tuple

import yaml


PROJECT_ROOT = pathlib.Path(__file__).resolve().parents[2]
DEFAULT_NAV_GRAPH = (
    PROJECT_ROOT / "office_maps" / "generated_maps" / "office" / "nav_graphs" / "0.yaml"
)
DEFAULT_OUTPUT_DIR = PROJECT_ROOT / "experiment_results" / "office_dynamic" / "analysis"
REQUIRED_WAYPOINTS = {
    "pantry",
    "hardware_2",
    "coe",
    "lounge",
    "tinyRobot1_charger",
    "tinyRobot2_charger",
}


@dataclass(frozen=True)
class Vertex:
    index: int
    x: float
    y: float
    name: str
    props: Dict[str, object]


@dataclass(frozen=True)
class Lane:
    index: int
    start: int
    end: int
    length_m: float
    props: Dict[str, object]


@dataclass
class PlanResult:
    algorithm: str
    start_name: str
    goal_name: str
    reachable: bool
    path_vertices: List[int]
    path_lanes: List[int]
    path_length_m: float
    node_count: int
    turn_count: int
    compute_ms: float
    closed_lanes: List[int]
    error: str = ""

    @property
    def traverses_closed_lane(self) -> bool:
        closed = set(self.closed_lanes)
        return any(lane in closed for lane in self.path_lanes)


class OfficeGraph:
    def __init__(self, vertices: List[Vertex], lanes: List[Lane]) -> None:
        self.vertices = vertices
        self.lanes = lanes
        self.name_to_index = {
            vertex.name: vertex.index for vertex in vertices if vertex.name
        }
        self.adjacency: Dict[int, List[Lane]] = {vertex.index: [] for vertex in vertices}
        for lane in lanes:
            self.adjacency[lane.start].append(lane)

    @classmethod
    def from_yaml(cls, path: pathlib.Path) -> "OfficeGraph":
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
        level = data["levels"]["L1"]

        vertices = []
        for index, raw in enumerate(level["vertices"]):
            props = dict(raw[2] or {})
            vertices.append(
                Vertex(
                    index=index,
                    x=float(raw[0]),
                    y=float(raw[1]),
                    name=str(props.get("name") or ""),
                    props=props,
                )
            )

        lanes = []
        for index, raw in enumerate(level["lanes"]):
            start = int(raw[0])
            end = int(raw[1])
            props = dict(raw[2] or {})
            lanes.append(
                Lane(
                    index=index,
                    start=start,
                    end=end,
                    length_m=euclidean(vertices[start], vertices[end]),
                    props=props,
                )
            )

        return cls(vertices, lanes)

    def vertex_name(self, index: int) -> str:
        name = self.vertices[index].name
        return name if name else f"v{index}"

    def lane_label(self, lane_index: int) -> str:
        lane = self.lanes[lane_index]
        return f"{lane_index}:{self.vertex_name(lane.start)}->{self.vertex_name(lane.end)}"

    def roles_for_vertex(self, vertex: Vertex) -> List[str]:
        roles = []
        if vertex.props.get("is_charger"):
            roles.append("charger")
        if vertex.props.get("is_holding_point"):
            roles.append("holding_point")
        if vertex.props.get("is_parking_spot"):
            roles.append("parking_spot")
        if "pickup_dispenser" in vertex.props:
            roles.append("pickup")
        if "dropoff_ingestor" in vertex.props:
            roles.append("dropoff")
        if vertex.name.startswith("patrol_"):
            roles.append("patrol")
        return roles


def euclidean(a: Vertex, b: Vertex) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)


def path_turn_count(graph: OfficeGraph, path_vertices: Sequence[int]) -> int:
    turns = 0
    for prev_idx, current_idx, next_idx in zip(
        path_vertices, path_vertices[1:], path_vertices[2:]
    ):
        prev_v = graph.vertices[prev_idx]
        current_v = graph.vertices[current_idx]
        next_v = graph.vertices[next_idx]
        v1 = (current_v.x - prev_v.x, current_v.y - prev_v.y)
        v2 = (next_v.x - current_v.x, next_v.y - current_v.y)
        n1 = math.hypot(*v1)
        n2 = math.hypot(*v2)
        if n1 < 1e-9 or n2 < 1e-9:
            continue
        cosine = max(-1.0, min(1.0, (v1[0] * v2[0] + v1[1] * v2[1]) / (n1 * n2)))
        angle_deg = math.degrees(math.acos(cosine))
        if angle_deg > 15.0:
            turns += 1
    return turns


def reconstruct_path(
    came_from: Dict[int, Tuple[int, int]], start_idx: int, goal_idx: int
) -> Tuple[List[int], List[int]]:
    vertices = [goal_idx]
    lanes = []
    current = goal_idx
    while current != start_idx:
        previous, lane_index = came_from[current]
        vertices.append(previous)
        lanes.append(lane_index)
        current = previous
    vertices.reverse()
    lanes.reverse()
    return vertices, lanes


def plan_route(
    graph: OfficeGraph,
    algorithm: str,
    start_name: str,
    goal_name: str,
    closed_lanes: Optional[Iterable[int]] = None,
) -> PlanResult:
    closed = set(closed_lanes or [])
    start_idx = graph.name_to_index[start_name]
    goal_idx = graph.name_to_index[goal_name]
    start_time = time.perf_counter()

    queue: List[Tuple[float, float, int]] = [(0.0, 0.0, start_idx)]
    best_cost = {start_idx: 0.0}
    came_from: Dict[int, Tuple[int, int]] = {}
    visited: Set[int] = set()

    while queue:
        _, current_cost, current_idx = heapq.heappop(queue)
        if current_idx in visited:
            continue
        visited.add(current_idx)

        if current_idx == goal_idx:
            compute_ms = (time.perf_counter() - start_time) * 1000.0
            path_vertices, path_lanes = reconstruct_path(came_from, start_idx, goal_idx)
            return PlanResult(
                algorithm=algorithm,
                start_name=start_name,
                goal_name=goal_name,
                reachable=True,
                path_vertices=path_vertices,
                path_lanes=path_lanes,
                path_length_m=current_cost,
                node_count=len(path_vertices),
                turn_count=path_turn_count(graph, path_vertices),
                compute_ms=compute_ms,
                closed_lanes=sorted(closed),
            )

        for lane in graph.adjacency[current_idx]:
            if lane.index in closed:
                continue
            next_cost = current_cost + lane.length_m
            if next_cost >= best_cost.get(lane.end, math.inf):
                continue
            best_cost[lane.end] = next_cost
            came_from[lane.end] = (current_idx, lane.index)
            if algorithm == "astar":
                priority = next_cost + euclidean(graph.vertices[lane.end], graph.vertices[goal_idx])
            elif algorithm == "dijkstra":
                priority = next_cost
            else:
                raise ValueError(f"Unsupported algorithm: {algorithm}")
            heapq.heappush(queue, (priority, next_cost, lane.end))

    compute_ms = (time.perf_counter() - start_time) * 1000.0
    return PlanResult(
        algorithm=algorithm,
        start_name=start_name,
        goal_name=goal_name,
        reachable=False,
        path_vertices=[],
        path_lanes=[],
        path_length_m=math.inf,
        node_count=0,
        turn_count=0,
        compute_ms=compute_ms,
        closed_lanes=sorted(closed),
        error="No route found after applying closed lanes",
    )


def baseline_scenarios() -> List[Tuple[str, str, str]]:
    return [
        ("delivery_primary", "pantry", "hardware_2"),
        ("delivery_alt_dropoff", "pantry", "coe"),
        ("patrol_support", "tinyRobot1_charger", "lounge"),
        ("recharge_to_pickup", "tinyRobot2_charger", "pantry"),
    ]


def choose_lane_closure_cases(graph: OfficeGraph) -> List[Dict[str, object]]:
    cases = []
    for scenario, start, goal in baseline_scenarios():
        base = plan_route(graph, "dijkstra", start, goal)
        if not base.reachable or not base.path_lanes:
            continue

        first = [base.path_lanes[0]]
        mid = [base.path_lanes[len(base.path_lanes) // 2]]
        first_two = base.path_lanes[:2] if len(base.path_lanes) >= 2 else first
        for suffix, closed in (
            ("close_first_lane", first),
            ("close_mid_lane", mid),
            ("close_first_two_lanes", first_two),
        ):
            cases.append(
                {
                    "scenario": f"{scenario}_{suffix}",
                    "start": start,
                    "goal": goal,
                    "baseline_length_m": base.path_length_m,
                    "closed_lanes": closed,
                }
            )
    return cases


def result_to_row(graph: OfficeGraph, scenario: str, result: PlanResult) -> Dict[str, object]:
    path_names = [graph.vertex_name(index) for index in result.path_vertices]
    closed_labels = [graph.lane_label(index) for index in result.closed_lanes]
    return {
        "scenario": scenario,
        "start": result.start_name,
        "goal": result.goal_name,
        "planner_algorithm": result.algorithm,
        "avoidance_strategy": "static_shortest_path"
        if not result.closed_lanes
        else "lane_closure_replan",
        "closed_lanes": ";".join(str(index) for index in result.closed_lanes),
        "closed_lane_labels": ";".join(closed_labels),
        "path_waypoints": " -> ".join(path_names),
        "path_lane_indices": ";".join(str(index) for index in result.path_lanes),
        "path_length_m": round(result.path_length_m, 3)
        if math.isfinite(result.path_length_m)
        else "",
        "node_count": result.node_count,
        "turn_count": result.turn_count,
        "compute_ms": round(result.compute_ms, 4),
        "reachable": result.reachable,
        "traverses_closed_lane": result.traverses_closed_lane,
        "error": result.error,
    }


def write_graph_extract(graph: OfficeGraph, output_dir: pathlib.Path) -> None:
    vertices_rows = []
    for vertex in graph.vertices:
        vertices_rows.append(
            {
                "kind": "vertex",
                "index": vertex.index,
                "name": vertex.name,
                "x": round(vertex.x, 3),
                "y": round(vertex.y, 3),
                "roles": ";".join(graph.roles_for_vertex(vertex)),
                "props_json": json.dumps(vertex.props, ensure_ascii=False, sort_keys=True),
            }
        )

    lane_rows = []
    for lane in graph.lanes:
        lane_rows.append(
            {
                "kind": "lane",
                "index": lane.index,
                "start": lane.start,
                "end": lane.end,
                "start_name": graph.vertex_name(lane.start),
                "end_name": graph.vertex_name(lane.end),
                "length_m": round(lane.length_m, 3),
                "door_name": lane.props.get("door_name", ""),
                "speed_limit": lane.props.get("speed_limit", ""),
                "props_json": json.dumps(lane.props, ensure_ascii=False, sort_keys=True),
            }
        )

    write_csv(
        output_dir / "office_nav_graph_extract.csv",
        vertices_rows + lane_rows,
        [
            "kind",
            "index",
            "name",
            "x",
            "y",
            "roles",
            "start",
            "end",
            "start_name",
            "end_name",
            "length_m",
            "door_name",
            "speed_limit",
            "props_json",
        ],
    )
    (output_dir / "office_nav_graph_extract.json").write_text(
        json.dumps(
            {
                "vertices": vertices_rows,
                "lanes": lane_rows,
                "required_waypoints_present": sorted(
                    REQUIRED_WAYPOINTS.intersection(graph.name_to_index)
                ),
            },
            indent=2,
            ensure_ascii=False,
        )
        + "\n",
        encoding="utf-8",
    )


def write_csv(path: pathlib.Path, rows: Sequence[Dict[str, object]], fields: Sequence[str]) -> None:
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(fields), extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def write_report(
    graph: OfficeGraph,
    output_dir: pathlib.Path,
    baseline_rows: Sequence[Dict[str, object]],
    closure_rows: Sequence[Dict[str, object]],
    missing_required: Set[str],
) -> None:
    primary_rows = [
        row
        for row in baseline_rows
        if row["scenario"] == "delivery_primary"
        and row["planner_algorithm"] in {"dijkstra", "astar"}
    ]
    closure_successes = [
        row
        for row in closure_rows
        if row["reachable"] is True and row["traverses_closed_lane"] is False
    ]

    lines = [
        "# Office Stage 2 Planning and Avoidance Comparison",
        "",
        "## Scope",
        "",
        "This evidence package is Office-only. It uses the imported Office RMF nav graph as the shared map input and does not use warehouse results as Office evidence.",
        "",
        "The current online Office stack is RMF nav graph planning and traffic scheduling, an `office_fleet_adapter` bridge, REST fleet manager path requests, and Gazebo `slotcar` execution for TinyRobot. It is not the removed old Nav2 Office costmap/controller path.",
        "",
        "The offline Dijkstra/A* runs below are reproducible baselines for Stage 2 comparison. They do not claim to replace the online RMF planner.",
        "",
        "## Static Map Checks",
        "",
        f"- Nav graph vertices: {len(graph.vertices)}",
        f"- Directed lanes: {len(graph.lanes)}",
        f"- Required Office waypoints present: {', '.join(sorted(REQUIRED_WAYPOINTS - missing_required))}",
    ]
    if missing_required:
        lines.append(f"- Missing required waypoints: {', '.join(sorted(missing_required))}")
    else:
        lines.append("- Missing required waypoints: none")

    lines.extend(
        [
            "",
            "## Path Planning Algorithms",
            "",
            "| Scenario | Algorithm | Reachable | Length (m) | Nodes | Turns | Compute (ms) | Path |",
            "| --- | --- | ---: | ---: | ---: | ---: | ---: | --- |",
        ]
    )
    for row in baseline_rows:
        lines.append(
            f"| {row['scenario']} | {row['planner_algorithm']} | {row['reachable']} | "
            f"{row['path_length_m']} | {row['node_count']} | {row['turn_count']} | "
            f"{row['compute_ms']} | {row['path_waypoints']} |"
        )

    lines.extend(
        [
            "",
            "Dijkstra and A* use the same Office graph and Euclidean lane length cost. A* adds an admissible straight-line heuristic from each waypoint to the goal, so it should match Dijkstra's shortest path length while usually expanding the graph with a stronger goal direction.",
            "",
            "## Avoidance and Conflict Strategies",
            "",
            "| Strategy | Evidence Level | What It Handles | Stage 2 Use |",
            "| --- | --- | --- | --- |",
            "| Static global shortest path | Offline quantified | Known Office topology without blocked lanes | Baseline route length and waypoint count |",
            "| Lane closure dynamic replanning | Offline quantified, matches RMF lane-closure concept | Door/lane outage or blocked corridor represented as closed directed lanes | Recompute route and verify closed lanes are not used |",
            "| RMF traffic schedule + blockade + slotcar feedback | Online architecture evidence | Multi-robot time-space conflict, waiting, and replan triggers | Current Office runtime strategy, documented separately from offline baselines |",
            "",
            "## Lane Closure Replanning Results",
            "",
            "| Scenario | Closed Lanes | Algorithm | Reachable | Detour Ratio | Uses Closed Lane | Length (m) | Path |",
            "| --- | --- | --- | ---: | ---: | ---: | ---: | --- |",
        ]
    )
    for row in closure_rows:
        lines.append(
            f"| {row['scenario']} | {row['closed_lane_labels']} | {row['planner_algorithm']} | "
            f"{row['reachable']} | {row['detour_ratio']} | {row['traverses_closed_lane']} | "
            f"{row['path_length_m']} | {row['path_waypoints']} |"
        )

    lines.extend(
        [
            "",
            "A lane-closure case passes when the recomputed path is reachable and `traverses_closed_lane=false`. If a case is unreachable, it is retained as a valid failure case because the closed lanes disconnect that OD pair in the current Office topology.",
            "",
            "## Runtime Chain Evidence",
            "",
            "- Office launch goes through `office/launch/office.launch.xml` into `office_demos` and `office_gz`.",
            "- `office_fleet_adapter` parses `office_maps/generated_maps/office/nav_graphs/0.yaml` and registers the TinyRobot fleet with RMF.",
            "- RMF sends `follow_new_path` waypoints to `RobotCommandHandle`, which forwards navigation commands through the fleet manager to `robot_path_requests`.",
            "- `lane_closure_requests` can close lanes and trigger `update_handle.replan()` when the current or remaining path is invalidated.",
            "- TinyRobot uses Gazebo `slotcar` execution, so local blockage feedback is not Nav2 DWA/costmap avoidance.",
            "",
            "## Existing Log Interpretation",
            "",
            "Current `office_dynamic/logs` entries can show that delivery requests were accepted or queued. They should not be presented as full Office completion evidence unless a later run also captures robot path execution and task completion. This report intentionally separates offline planning evidence from online runtime evidence.",
            "",
            "## Acceptance Summary",
            "",
            f"- 2+ path planners quantified: {'yes' if primary_rows else 'no'}",
            f"- Lane closure replanning cases generated: {len(closure_rows)}",
            f"- Passing reachable lane-closure cases: {len(closure_successes)}",
            "- RMF traffic schedule/blockade documented as the current online conflict-management strategy.",
        ]
    )

    (output_dir / "office_stage2_planning_avoidance_report.md").write_text(
        "\n".join(lines) + "\n", encoding="utf-8"
    )


def generate(output_dir: pathlib.Path, nav_graph: pathlib.Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    graph = OfficeGraph.from_yaml(nav_graph)
    missing_required = REQUIRED_WAYPOINTS - set(graph.name_to_index)
    if missing_required:
        raise RuntimeError(f"Missing required Office waypoints: {sorted(missing_required)}")

    write_graph_extract(graph, output_dir)

    baseline_rows: List[Dict[str, object]] = []
    for scenario, start, goal in baseline_scenarios():
        for algorithm in ("dijkstra", "astar"):
            result = plan_route(graph, algorithm, start, goal)
            baseline_rows.append(result_to_row(graph, scenario, result))

    baseline_fields = [
        "scenario",
        "start",
        "goal",
        "planner_algorithm",
        "avoidance_strategy",
        "closed_lanes",
        "closed_lane_labels",
        "path_waypoints",
        "path_lane_indices",
        "path_length_m",
        "node_count",
        "turn_count",
        "compute_ms",
        "reachable",
        "traverses_closed_lane",
        "error",
    ]
    write_csv(output_dir / "office_route_baseline.csv", baseline_rows, baseline_fields)

    closure_rows: List[Dict[str, object]] = []
    for case in choose_lane_closure_cases(graph):
        for algorithm in ("dijkstra", "astar"):
            result = plan_route(
                graph,
                algorithm,
                str(case["start"]),
                str(case["goal"]),
                closed_lanes=case["closed_lanes"],
            )
            row = result_to_row(graph, str(case["scenario"]), result)
            baseline_length = float(case["baseline_length_m"])
            if result.reachable and baseline_length > 0.0:
                row["detour_ratio"] = round(result.path_length_m / baseline_length, 3)
            else:
                row["detour_ratio"] = ""
            closure_rows.append(row)

    closure_fields = baseline_fields + ["detour_ratio"]
    write_csv(output_dir / "office_lane_closure_cases.csv", closure_rows, closure_fields)
    write_report(graph, output_dir, baseline_rows, closure_rows, missing_required)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate Office path planning and avoidance comparison evidence."
    )
    parser.add_argument("--nav-graph", type=pathlib.Path, default=DEFAULT_NAV_GRAPH)
    parser.add_argument("--output-dir", type=pathlib.Path, default=DEFAULT_OUTPUT_DIR)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    generate(args.output_dir, args.nav_graph)
    print(f"Generated Office planning evidence in: {args.output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

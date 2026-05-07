#!/usr/bin/env python3

import argparse
import csv
import json
import pathlib
import time
from typing import Dict, List

import rclpy

from _course_task_utils import TaskDispatchNode, dump_result


def parse_args():
    parser = argparse.ArgumentParser(
        description="Run repeated preset course tasks and record metrics for planner and avoidance comparisons."
    )
    parser.add_argument("--scene", choices=["warehouse", "office"], required=True)
    parser.add_argument("--task", choices=["delivery", "patrol"], required=True)
    parser.add_argument("--trials", type=int, default=3)
    parser.add_argument("--planner-profile", choices=["configured", "navfn_astar", "smac_2d"], default="configured")
    parser.add_argument("--avoidance-profile", choices=["baseline_costmap", "collision_monitor"], default="collision_monitor")
    parser.add_argument("--output-dir", default="experiment_results")
    parser.add_argument("--notes", default="")
    return parser.parse_args()


def write_outputs(records: List[Dict[str, object]], output_dir: pathlib.Path, stem: str):
    output_dir.mkdir(parents=True, exist_ok=True)
    jsonl_path = output_dir / f"{stem}.jsonl"
    csv_path = output_dir / f"{stem}.csv"

    with jsonl_path.open("w", encoding="utf-8") as stream:
        for record in records:
            stream.write(json.dumps(record, ensure_ascii=False) + "\n")

    fieldnames = [
        "trial",
        "scene",
        "task",
        "planner_profile",
        "avoidance_profile",
        "accepted",
        "status",
        "task_status",
        "elapsed_sec",
        "route",
        "error",
        "notes",
    ]
    with csv_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for record in records:
            writer.writerow({field: record.get(field, "") for field in fieldnames})

    return jsonl_path, csv_path


def main():
    args = parse_args()
    records: List[Dict[str, object]] = []
    rclpy.init()
    node = TaskDispatchNode()
    try:
        for trial in range(1, args.trials + 1):
            started_at = time.strftime("%Y-%m-%dT%H:%M:%S%z")
            try:
                result = node.dispatch(args.scene, args.task)
                payload = json.loads(dump_result(result))
                task_spec = payload.get("task_spec", {})
                record: Dict[str, object] = {
                    "trial": trial,
                    "started_at": started_at,
                    "scene": args.scene,
                    "task": args.task,
                    "planner_profile": args.planner_profile,
                    "avoidance_profile": args.avoidance_profile,
                    "accepted": payload.get("accepted", False),
                    "status": payload.get("status", ""),
                    "task_status": payload.get("task_status", ""),
                    "elapsed_sec": payload.get("elapsed_sec", 0.0),
                    "route": " -> ".join(task_spec.get("route", [])),
                    "error": "",
                    "notes": args.notes,
                }
            except Exception as exc:
                record = {
                    "trial": trial,
                    "started_at": started_at,
                    "scene": args.scene,
                    "task": args.task,
                    "planner_profile": args.planner_profile,
                    "avoidance_profile": args.avoidance_profile,
                    "accepted": False,
                    "status": "ERROR",
                    "task_status": "ERROR",
                    "elapsed_sec": 0.0,
                    "route": "",
                    "error": f"{type(exc).__name__}: {exc}",
                    "notes": args.notes,
                }
            records.append(record)
            print(json.dumps(record, ensure_ascii=False))
    finally:
        node.destroy_node()
        rclpy.shutdown()

    stem = f"{args.scene}_{args.task}_{args.planner_profile}_{args.avoidance_profile}"
    jsonl_path, csv_path = write_outputs(records, pathlib.Path(args.output_dir), stem)
    print(f"Saved JSONL: {jsonl_path}")
    print(f"Saved CSV: {csv_path}")


if __name__ == "__main__":
    main()

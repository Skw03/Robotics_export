#!/usr/bin/env python3

import argparse
import pathlib

import rclpy

from _course_task_utils import TaskDispatchNode, dump_result


def parse_args():
    parser = argparse.ArgumentParser(description="Dispatch a preset course task to the active scenario action server.")
    parser.add_argument("--scene", choices=["warehouse", "office"], default="warehouse")
    parser.add_argument("--task", choices=["delivery", "patrol"], default="delivery")
    parser.add_argument("--save-json", help="Optional path to save the dispatch result as JSON")
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = TaskDispatchNode()
    try:
        result = node.dispatch(args.scene, args.task)
        payload = dump_result(result)
        print(payload)
        if args.save_json:
            output_path = pathlib.Path(args.save_json)
            output_path.write_text(payload + "\n", encoding="utf-8")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

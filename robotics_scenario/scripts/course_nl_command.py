#!/usr/bin/env python3

import argparse


def infer_scene(text: str) -> str:
    lowered = text.lower()
    if "office" in lowered or "办公" in text or "办公室" in text or "办公区" in text:
        return "office"
    return "warehouse"


def infer_task(text: str) -> str:
    lowered = text.lower()
    patrol_keywords = ["patrol", "loop", "巡检", "巡航", "巡视", "巡逻", "检查"]
    if any(keyword in lowered or keyword in text for keyword in patrol_keywords):
        return "patrol"
    return "delivery"


def main():
    parser = argparse.ArgumentParser(description="Parse a natural-language command into a preset course scenario task.")
    parser.add_argument("text", help="Natural language command in Chinese or English")
    args = parser.parse_args()

    scene = infer_scene(args.text)
    task = infer_task(args.text)

    import rclpy

    from _course_task_utils import TaskDispatchNode, dump_result

    rclpy.init()
    node = TaskDispatchNode()
    try:
        result = node.dispatch(scene, task)
        print(dump_result(result))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

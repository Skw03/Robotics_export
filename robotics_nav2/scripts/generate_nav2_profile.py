#!/usr/bin/env python3

import argparse
import pathlib


def parse_args():
    parser = argparse.ArgumentParser(description="Generate Nav2 parameter profiles for course comparison experiments.")
    parser.add_argument("--base", required=True, help="Base Nav2 YAML file")
    parser.add_argument("--planner-profile", choices=["navfn_astar", "smac_2d"], required=True)
    parser.add_argument("--avoidance-profile", choices=["baseline_costmap", "collision_monitor", "stage2_demo"], required=True)
    parser.add_argument("--output", required=True, help="Output YAML path")
    return parser.parse_args()


def replace_planner_profile(text: str, planner_profile: str) -> str:
    if planner_profile == "navfn_astar":
        return text.replace('planner_plugins: ["GridBased", "Smac2D"]', 'planner_plugins: ["GridBased"]')
    return text.replace('planner_plugins: ["GridBased", "Smac2D"]', 'planner_plugins: ["Smac2D"]')


def replace_avoidance_profile(text: str, avoidance_profile: str) -> str:
    if avoidance_profile == "collision_monitor":
        return text
    text = text.replace(
        'polygons: ["PolygonStop", "PolygonSlow", "FootprintApproach"]',
        'polygons: []',
    )
    if avoidance_profile == "stage2_demo":
        text = text.replace("desired_linear_vel: 0.55", "desired_linear_vel: 0.35")
        text = text.replace("max_velocity: [0.80, 0.0, 1.4]", "max_velocity: [0.45, 0.0, 1.0]")
        text = text.replace("use_collision_detection: true", "use_collision_detection: false")
        text = text.replace("xy_goal_tolerance: 0.12", "xy_goal_tolerance: 0.20")
    return text


def main():
    args = parse_args()
    base = pathlib.Path(args.base)
    output = pathlib.Path(args.output)
    text = base.read_text(encoding="utf-8")
    text = replace_planner_profile(text, args.planner_profile)
    text = replace_avoidance_profile(text, args.avoidance_profile)
    header = (
        f"# Generated from {base.name}\n"
        f"# planner_profile: {args.planner_profile}\n"
        f"# avoidance_profile: {args.avoidance_profile}\n"
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(header + text, encoding="utf-8")
    print(output)


if __name__ == "__main__":
    main()

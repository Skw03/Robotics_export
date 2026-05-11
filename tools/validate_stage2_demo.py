#!/usr/bin/env python3

import argparse
import ast
import pathlib
import sys
from typing import Dict, Iterable, Tuple

import yaml


ROOT = pathlib.Path(__file__).resolve().parents[1]
WAREHOUSE_MAP = ROOT / "robotics_nav2" / "map" / "warehouse_map.yaml"
WAREHOUSE_GOALS = ROOT / "robotics_scenario" / "param" / "warehouse_semantic_goals.yaml"
OFFICE_MAP = ROOT / "robotics_nav2" / "map" / "office_map.yaml"
OFFICE_GOALS = ROOT / "robotics_scenario" / "param" / "office_semantic_goals.yaml"
TASK_UTILS = ROOT / "robotics_scenario" / "scripts"


def load_yaml(path: pathlib.Path) -> Dict[str, object]:
    with path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ValueError(f"{path} did not contain a YAML mapping")
    return data


def map_bounds(map_yaml: pathlib.Path) -> Tuple[float, float, float, float]:
    data = load_yaml(map_yaml)
    origin = data["origin"]
    resolution = float(data["resolution"])
    image_path = map_yaml.parent / str(data["image"])

    from PIL import Image

    with Image.open(image_path) as image:
        width, height = image.size
    min_x = float(origin[0])
    min_y = float(origin[1])
    return min_x, min_x + width * resolution, min_y, min_y + height * resolution


def iter_goals(goals_yaml: pathlib.Path) -> Iterable[Tuple[str, float, float]]:
    data = load_yaml(goals_yaml)
    goals = data.get("semantic_goals", {})
    if not isinstance(goals, dict):
        raise ValueError("semantic_goals must be a mapping")
    for name, spec in goals.items():
        if not isinstance(spec, dict):
            raise ValueError(f"Goal {name} must be a mapping")
        yield name, float(spec["x"]), float(spec["y"])


def validate_warehouse_bounds() -> int:
    min_x, max_x, min_y, max_y = map_bounds(WAREHOUSE_MAP)
    errors = []
    for name, x, y in iter_goals(WAREHOUSE_GOALS):
        if not (min_x <= x <= max_x and min_y <= y <= max_y):
            errors.append(f"{name}: ({x:.2f}, {y:.2f}) outside x=[{min_x:.2f},{max_x:.2f}], y=[{min_y:.2f},{max_y:.2f}]")
    if errors:
        print("Warehouse waypoint bounds check failed:", file=sys.stderr)
        for error in errors:
            print(f"  - {error}", file=sys.stderr)
        return 1
    print(f"Warehouse waypoint bounds check passed: x=[{min_x:.2f},{max_x:.2f}], y=[{min_y:.2f},{max_y:.2f}]")
    return 0


def validate_office_free_space() -> int:
    data = load_yaml(OFFICE_MAP)
    origin = data["origin"]
    resolution = float(data["resolution"])
    image_path = OFFICE_MAP.parent / str(data["image"])

    from PIL import Image

    with Image.open(image_path) as image:
        occupancy = image.convert("L")
        width, height = occupancy.size
        min_x = float(origin[0])
        min_y = float(origin[1])
        max_x = min_x + width * resolution
        max_y = min_y + height * resolution

        errors = []
        for name, x, y in iter_goals(OFFICE_GOALS):
            if not (min_x <= x <= max_x and min_y <= y <= max_y):
                errors.append(f"{name}: ({x:.2f}, {y:.2f}) outside x=[{min_x:.2f},{max_x:.2f}], y=[{min_y:.2f},{max_y:.2f}]")
                continue
            pixel_x = int((x - min_x) / resolution)
            pixel_y = height - 1 - int((y - min_y) / resolution)
            pixel_x = min(max(pixel_x, 0), width - 1)
            pixel_y = min(max(pixel_y, 0), height - 1)
            gray = occupancy.getpixel((pixel_x, pixel_y))
            if gray < 250:
                errors.append(f"{name}: ({x:.2f}, {y:.2f}) maps to non-free office pixel gray={gray}")

    if errors:
        print("Office waypoint free-space check failed:", file=sys.stderr)
        for error in errors:
            print(f"  - {error}", file=sys.stderr)
        return 1
    print(f"Office waypoint free-space check passed: x=[{min_x:.2f},{max_x:.2f}], y=[{min_y:.2f},{max_y:.2f}]")
    return 0


def validate_demo_routes() -> int:
    task_utils_path = TASK_UTILS / "_course_task_utils.py"
    tree = ast.parse(task_utils_path.read_text(encoding="utf-8"), filename=str(task_utils_path))
    constants = {}
    for node in tree.body:
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Name) and target.id in {"SEMANTIC_POSES", "TASK_PRESETS"}:
                    constants[target.id] = ast.literal_eval(node.value)
        elif isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name):
            if node.target.id in {"SEMANTIC_POSES", "TASK_PRESETS"}:
                constants[node.target.id] = ast.literal_eval(node.value)

    semantic_poses = constants["SEMANTIC_POSES"]
    task_presets = constants["TASK_PRESETS"]

    errors = []
    for scene, tasks in task_presets.items():
        if "demo" not in tasks:
            errors.append(f"{scene}: missing demo task")
            continue
        for waypoint in tasks["demo"]["route"]:
            if waypoint not in semantic_poses[scene]:
                errors.append(f"{scene}: demo route references missing waypoint {waypoint}")
    if errors:
        print("Demo route validation failed:", file=sys.stderr)
        for error in errors:
            print(f"  - {error}", file=sys.stderr)
        return 1
    print("Demo route validation passed for warehouse and office")
    return 0


def parse_args():
    parser = argparse.ArgumentParser(description="Validate stage-2 demo route assumptions.")
    parser.add_argument("--warehouse-bounds", action="store_true", help="Check warehouse semantic goals are inside the map")
    parser.add_argument("--office-free-space", action="store_true", help="Check office semantic goals are inside the map and on free-space pixels")
    parser.add_argument("--demo-routes", action="store_true", help="Check demo routes reference defined semantic goals")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    run_all = not args.warehouse_bounds and not args.office_free_space and not args.demo_routes
    status = 0
    if run_all or args.warehouse_bounds:
        status |= validate_warehouse_bounds()
    if run_all or args.office_free_space:
        status |= validate_office_free_space()
    if run_all or args.demo_routes:
        status |= validate_demo_routes()
    return status


if __name__ == "__main__":
    raise SystemExit(main())

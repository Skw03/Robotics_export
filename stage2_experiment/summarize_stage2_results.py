#!/usr/bin/env python3
import csv
import glob
import json
import os
from collections import defaultdict


def mean(values):
    return sum(values) / len(values) if values else 0.0


def summarize_nav(nav_dir, out_csv):
    grouped = defaultdict(list)
    for path in glob.glob(os.path.join(nav_dir, "*.csv")):
        with open(path, encoding="utf-8") as f:
            for row in csv.DictReader(f):
                key = (
                    row.get("scene", ""),
                    row.get("task", ""),
                    row.get("planner_profile", ""),
                    row.get("avoidance_profile", ""),
                )
                grouped[key].append(row)

    rows = []
    for (scene, task, planner, avoidance), recs in sorted(grouped.items()):
        total = len(recs)
        success = sum(1 for r in recs if str(r.get("accepted", "")).lower() == "true" and r.get("status", "") not in ("ERROR", "TIMEOUT"))
        elapsed = [float(r.get("elapsed_sec", 0) or 0) for r in recs]
        path_len = [float(r.get("path_length_m", 0) or 0) for r in recs]
        min_obs = [float(r.get("min_obstacle_dist_m", -1) or -1) for r in recs if float(r.get("min_obstacle_dist_m", -1) or -1) >= 0]
        near_col = [float(r.get("near_collision_events", 0) or 0) for r in recs]
        rows.append({
            "scene": scene,
            "task": task,
            "planner_profile": planner,
            "avoidance_profile": avoidance,
            "trials": total,
            "success_rate": round(success / total, 4) if total else 0.0,
            "elapsed_sec_mean": round(mean(elapsed), 3),
            "path_length_m_mean": round(mean(path_len), 3),
            "min_obstacle_dist_m_mean": round(mean(min_obs), 3) if min_obs else -1.0,
            "near_collision_events_mean": round(mean(near_col), 3),
        })

    with open(out_csv, "w", encoding="utf-8", newline="") as f:
        fieldnames = list(rows[0].keys()) if rows else [
            "scene", "task", "planner_profile", "avoidance_profile", "trials",
            "success_rate", "elapsed_sec_mean", "path_length_m_mean",
            "min_obstacle_dist_m_mean", "near_collision_events_mean"
        ]
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def summarize_llm(llm_dir, out_json):
    files = sorted(glob.glob(os.path.join(llm_dir, "*.json")))
    total = 0
    fallback = 0
    errors = defaultdict(int)
    latencies = []
    for path in files:
        try:
            with open(path, encoding="utf-8") as f:
                data = json.load(f)
        except Exception:
            continue
        parsed = data.get("parsed", {})
        total += 1
        parser = parsed.get("parser", "")
        if "fallback" in parser:
            fallback += 1
        err = parsed.get("error", "")
        if err:
            errors[err.split(":")[0]] += 1
        latencies.append(float(parsed.get("latency_sec", 0) or 0))

    summary = {
        "samples": total,
        "fallback_count": fallback,
        "fallback_rate": round(fallback / total, 4) if total else 0.0,
        "latency_sec_mean": round(mean(latencies), 4) if latencies else 0.0,
        "error_type_counts": dict(sorted(errors.items())),
    }
    with open(out_json, "w", encoding="utf-8") as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)


def main():
    project_root = os.environ.get("PROJECT_ROOT", os.getcwd())
    nav_dir = os.path.join(project_root, "experiment_results", "stage2_matrix", "nav")
    llm_dir = os.path.join(project_root, "experiment_results", "llm", "runs")
    out_dir = os.path.join(project_root, "experiment_results", "summary")
    os.makedirs(out_dir, exist_ok=True)

    summarize_nav(nav_dir, os.path.join(out_dir, "nav_summary.csv"))
    summarize_llm(llm_dir, os.path.join(out_dir, "llm_summary.json"))
    print(f"Saved summary files under: {out_dir}")


if __name__ == "__main__":
    main()

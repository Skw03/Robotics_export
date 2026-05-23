#!/usr/bin/env python3
import argparse
import json
import os
import subprocess
import sys
import time
import urllib.error
import urllib.request


TASKS = {"delivery", "patrol"}


def fallback_parse(text):
    lowered = text.lower()
    patrol_words = ["patrol", "inspect", "inspection", "巡检", "巡视", "巡逻", "检查"]
    delivery_words = ["deliver", "delivery", "file", "hardware", "送", "配送", "文件", "硬件"]
    score = {"patrol": 0, "delivery": 0}
    score["patrol"] += sum(1 for word in patrol_words if word in lowered or word in text)
    score["delivery"] += sum(1 for word in delivery_words if word in lowered or word in text)
    if score["patrol"] > score["delivery"]:
        task = "patrol"
    elif score["delivery"] > 0:
        task = "delivery"
    else:
        task = "patrol"
    confidence = 0.86 if max(score.values()) > 0 else 0.45
    return {
        "task": task,
        "confidence": confidence,
        "parser": "keyword_fallback",
        "reason": "matched Office delivery/patrol keywords" if max(score.values()) > 0 else "defaulted to patrol for ambiguous Office command",
        "error": None,
    }


def openai_parse(text, model):
    api_key = os.environ.get("OPENAI_API_KEY")
    if not api_key:
        raise RuntimeError("OPENAI_API_KEY is not set")

    schema = {
        "name": "office_task_parse",
        "schema": {
            "type": "object",
            "additionalProperties": False,
            "properties": {
                "task": {"type": "string", "enum": ["delivery", "patrol"]},
                "confidence": {"type": "number", "minimum": 0, "maximum": 1},
                "reason": {"type": "string"},
            },
            "required": ["task", "confidence", "reason"],
        },
        "strict": True,
    }
    body = {
        "model": model,
        "input": [
            {
                "role": "system",
                "content": (
                    "Parse an Office robot service command. Only choose delivery or patrol. "
                    "Reject warehouse or unsupported robot tasks by mapping to the closest Office task with low confidence."
                ),
            },
            {"role": "user", "content": text},
        ],
        "text": {"format": {"type": "json_schema", **schema}},
    }
    request = urllib.request.Request(
        "https://api.openai.com/v1/responses",
        data=json.dumps(body).encode("utf-8"),
        headers={
            "Authorization": f"Bearer {api_key}",
            "Content-Type": "application/json",
        },
        method="POST",
    )
    with urllib.request.urlopen(request, timeout=20) as response:
        payload = json.loads(response.read().decode("utf-8"))

    chunks = []
    for item in payload.get("output", []):
        for content in item.get("content", []):
            if content.get("type") in {"output_text", "text"}:
                chunks.append(content.get("text", ""))
    if not chunks:
        raise RuntimeError("OpenAI response did not contain output text")
    parsed = json.loads("".join(chunks))
    parsed["parser"] = "openai_responses_rest"
    parsed["error"] = None
    return parsed


def command_for_task(task):
    if task == "delivery":
        return ["ros2", "run", "office", "dispatch_delivery", "--use_sim_time"]
    if task == "patrol":
        return ["ros2", "run", "office", "dispatch_patrol", "--use_sim_time"]
    raise ValueError(f"Unsupported task: {task}")


def main():
    parser = argparse.ArgumentParser(description="Office-only natural-language task parser.")
    parser.add_argument("command", nargs="+", help="Natural-language Office command.")
    parser.add_argument("--execute", action="store_true", help="Dispatch the parsed Office task.")
    parser.add_argument("--force-fallback", action="store_true")
    parser.add_argument("--save-json", help="Write parse and execution evidence to a JSON file.")
    parser.add_argument("--model", default=os.environ.get("OPENAI_MODEL", "gpt-4o-mini"))
    args = parser.parse_args()

    text = " ".join(args.command)
    started = time.time()
    try:
        parsed = fallback_parse(text) if args.force_fallback else openai_parse(text, args.model)
    except Exception as exc:
        parsed = fallback_parse(text)
        parsed["error"] = f"{type(exc).__name__}: {exc}"
    elapsed_ms = round((time.time() - started) * 1000, 2)

    task = parsed.get("task")
    if task not in TASKS:
        parsed = fallback_parse(text)
        task = parsed["task"]
        parsed["error"] = "model returned unsupported task; fallback parser used"

    dispatch_command = command_for_task(task)
    result = {
        "input": text,
        "parsed": parsed,
        "dry_run": not args.execute,
        "dispatch_command": dispatch_command,
        "parse_latency_ms": elapsed_ms,
    }

    if args.execute:
        dispatch_started = time.time()
        proc = subprocess.run(dispatch_command, text=True, capture_output=True)
        result["execution"] = {
            "returncode": proc.returncode,
            "elapsed_sec": round(time.time() - dispatch_started, 3),
            "stdout": proc.stdout,
            "stderr": proc.stderr,
        }

    if args.save_json:
        path = os.path.abspath(args.save_json)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            json.dump(result, f, ensure_ascii=False, indent=2)

    print(json.dumps(result, ensure_ascii=False, indent=2))
    if args.execute and result.get("execution", {}).get("returncode", 1) != 0:
        return result["execution"]["returncode"]
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

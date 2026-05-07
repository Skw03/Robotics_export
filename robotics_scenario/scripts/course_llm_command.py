#!/usr/bin/env python3

import argparse
import json
import os
import time
from dataclasses import asdict, dataclass
from typing import Any, Dict, Optional

from course_nl_command import infer_scene, infer_task


VALID_TASKS = {
    "warehouse": {"delivery", "patrol"},
    "office": {"delivery", "patrol"},
}


SYSTEM_PROMPT = """You are the semantic planning layer for a ROS 2 indoor mobile robot.
Map the user's command to exactly one executable preset task.
Valid scenes: warehouse, office.
Valid tasks: delivery, patrol.
Return JSON only. Do not invent scenes, tasks, or route names."""


TASK_SCHEMA: Dict[str, Any] = {
    "type": "object",
    "additionalProperties": False,
    "properties": {
        "scene": {"type": "string", "enum": ["warehouse", "office"]},
        "task": {"type": "string", "enum": ["delivery", "patrol"]},
        "confidence": {"type": "number", "minimum": 0, "maximum": 1},
        "rationale": {"type": "string"},
    },
    "required": ["scene", "task", "confidence", "rationale"],
}


@dataclass
class ParsedCommand:
    scene: str
    task: str
    confidence: float
    rationale: str
    parser: str
    latency_sec: float
    error: str = ""


def fallback_parse(text: str, start_time: Optional[float] = None, error: str = "") -> ParsedCommand:
    start = start_time if start_time is not None else time.time()
    scene = infer_scene(text)
    task = infer_task(text)
    return ParsedCommand(
        scene=scene,
        task=task,
        confidence=0.55,
        rationale="Keyword fallback parser mapped the command to a preset task.",
        parser="keyword_fallback",
        latency_sec=time.time() - start,
        error=error,
    )


def parse_json_payload(raw: str) -> Dict[str, Any]:
    raw = raw.strip()
    if raw.startswith("```"):
        raw = raw.strip("`")
        raw = raw.split("\n", 1)[-1]
    return json.loads(raw)


def parse_with_openai_sdk(text: str, model: str) -> Dict[str, Any]:
    from openai import OpenAI  # type: ignore

    client = OpenAI()
    response = client.responses.create(
        model=model,
        input=[
            {"role": "system", "content": SYSTEM_PROMPT},
            {"role": "user", "content": text},
        ],
        text={
            "format": {
                "type": "json_schema",
                "name": "robot_task_command",
                "schema": TASK_SCHEMA,
                "strict": True,
            }
        },
    )
    return parse_json_payload(response.output_text)


def parse_with_openai_rest(text: str, model: str) -> Dict[str, Any]:
    import urllib.request

    api_key = os.environ["OPENAI_API_KEY"]
    payload = {
        "model": model,
        "input": [
            {"role": "system", "content": SYSTEM_PROMPT},
            {"role": "user", "content": text},
        ],
        "text": {
            "format": {
                "type": "json_schema",
                "name": "robot_task_command",
                "schema": TASK_SCHEMA,
                "strict": True,
            }
        },
    }
    request = urllib.request.Request(
        "https://api.openai.com/v1/responses",
        data=json.dumps(payload).encode("utf-8"),
        headers={
            "Authorization": f"Bearer {api_key}",
            "Content-Type": "application/json",
        },
        method="POST",
    )
    with urllib.request.urlopen(request, timeout=30) as response:
        data = json.loads(response.read().decode("utf-8"))

    chunks = []
    for item in data.get("output", []):
        for content in item.get("content", []):
            if content.get("type") in ("output_text", "text"):
                chunks.append(content.get("text", ""))
    return parse_json_payload("".join(chunks))


def parse_command(text: str, model: str, force_fallback: bool = False) -> ParsedCommand:
    start = time.time()
    if force_fallback or not os.environ.get("OPENAI_API_KEY"):
        reason = "OPENAI_API_KEY is not set" if not force_fallback else "fallback forced"
        return fallback_parse(text, start, reason)

    try:
        try:
            payload = parse_with_openai_sdk(text, model)
            parser = "openai_responses_sdk"
        except ImportError:
            payload = parse_with_openai_rest(text, model)
            parser = "openai_responses_rest"

        scene = payload["scene"]
        task = payload["task"]
        if scene not in VALID_TASKS or task not in VALID_TASKS[scene]:
            raise ValueError(f"Model returned unsupported task: scene={scene}, task={task}")

        return ParsedCommand(
            scene=scene,
            task=task,
            confidence=float(payload.get("confidence", 0.0)),
            rationale=str(payload.get("rationale", "")),
            parser=parser,
            latency_sec=time.time() - start,
        )
    except Exception as exc:
        return fallback_parse(text, start, f"{type(exc).__name__}: {exc}")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Parse natural language with an OpenAI LLM semantic layer and dispatch a preset ROS 2 course task."
    )
    parser.add_argument("text", help="Natural language command in Chinese or English")
    parser.add_argument("--model", default=os.environ.get("OPENAI_MODEL", "gpt-5.4-mini"))
    parser.add_argument("--dry-run", action="store_true", help="Only print the parsed task; do not dispatch")
    parser.add_argument("--force-fallback", action="store_true", help="Use the local keyword parser even if an API key is set")
    parser.add_argument("--save-json", help="Optional path to save parse and dispatch results")
    return parser.parse_args()


def main():
    args = parse_args()
    parsed = parse_command(args.text, args.model, args.force_fallback)
    output: Dict[str, Any] = {
        "input": args.text,
        "parsed": asdict(parsed),
        "model": args.model if parsed.parser.startswith("openai") else "",
    }

    if not args.dry_run:
        import rclpy

        from _course_task_utils import TaskDispatchNode, dump_result

        rclpy.init()
        node = TaskDispatchNode()
        try:
            result = node.dispatch(parsed.scene, parsed.task)
            output["dispatch"] = json.loads(dump_result(result))
        finally:
            node.destroy_node()
            rclpy.shutdown()

    payload = json.dumps(output, indent=2, ensure_ascii=False)
    print(payload)
    if args.save_json:
        with open(args.save_json, "w", encoding="utf-8") as stream:
            stream.write(payload + "\n")


if __name__ == "__main__":
    main()

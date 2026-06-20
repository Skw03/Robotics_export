#!/usr/bin/env python3
"""Office-only natural-language task parser with LLM integration.

Supports multiple LLM providers:
  - openai:       OpenAI API (Responses endpoint) with structured JSON output.
  - openai_compat: OpenAI-compatible Chat Completions API (智谱, DeepSeek, etc.).
  - mock:         Deterministic mock responses for testing without an API key.
  - keyword_fallback: Local keyword matching (always available as last resort).

Configuration priority: environment variable > config.toml > default.
If the API key is empty, mock mode is used automatically.

Supported providers:
  - openai:  Uses /responses endpoint with json_schema structured output.
  - openai_compat: Uses /chat/completions endpoint (智谱, DeepSeek, Moonshot, etc.).
  - mock:    No API call, keyword-based selection with mock confidence.

Environment variables:
  LLM_API_KEY     - API key (overrides config.toml)
  LLM_MODEL       - Model name (overrides config.toml)
  LLM_BASE_URL    - Base URL (overrides config.toml)
  LLM_PROVIDER    - Provider: openai | openai_compat | mock (overrides config.toml)

Legacy env vars (still supported):
  OPENAI_API_KEY, OPENAI_MODEL, OPENAI_BASE_URL, OPENAI_PROVIDER
"""

import argparse
import json
import os
import subprocess
import sys
import time
import urllib.error
import urllib.request

# ---------------------------------------------------------------------------
# Configuration helpers
# ---------------------------------------------------------------------------

# Search for config.toml in multiple locations:
# 1. Resolved source tree (handles symlinks from --symlink-install)
# 2. Install prefix (if not using symlinks)
# 3. Current working directory
# 4. Workspace root
_SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
_CONFIG_SEARCH_PATHS = [
    os.path.abspath(os.path.join(_SCRIPT_DIR, os.pardir, os.pardir, os.pardir, os.pardir, os.pardir)),
    os.path.abspath(os.path.join(_SCRIPT_DIR, os.pardir, os.pardir)),
    os.getcwd(),
    os.path.abspath(os.path.join(os.getcwd(), "src", "Robotics", "Robotics_export")),
]

_CONFIG_PATH = None
for _search_dir in _CONFIG_SEARCH_PATHS:
    _candidate = os.path.join(_search_dir, "config.toml")
    if os.path.isfile(_candidate):
        _CONFIG_PATH = _candidate
        break


def _load_toml(path):
    """Minimal TOML parser (no third-party dependency)."""
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
                # Remove inline comment for unquoted values
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


def _get_config():
    """Load config.toml; return empty dict if file is missing."""
    if os.path.isfile(_CONFIG_PATH):
        try:
            return _load_toml(_CONFIG_PATH)
        except Exception:
            return {}
    return {}


def _resolve_llm_config():
    """Resolve LLM settings with priority: env var > config.toml > default."""
    cfg = _get_config().get("llm", {})
    mock_cfg = cfg.get("mock", {})

    # New env vars take priority; fall back to legacy OPENAI_* vars
    api_key = (
        os.environ.get("LLM_API_KEY")
        or os.environ.get("OPENAI_API_KEY")
        or cfg.get("api_key", "")
    )
    model = (
        os.environ.get("LLM_MODEL")
        or os.environ.get("OPENAI_MODEL")
        or cfg.get("model", "gpt-4o-mini")
    )
    base_url = (
        os.environ.get("LLM_BASE_URL")
        or os.environ.get("OPENAI_BASE_URL")
        or cfg.get("base_url", "https://api.openai.com/v1")
    )
    provider_env = (
        os.environ.get("LLM_PROVIDER")
        or os.environ.get("OPENAI_PROVIDER")
    )
    timeout = cfg.get("timeout_sec", 20)

    # If api_key is empty, force mock mode
    if not api_key:
        provider = "mock"
    elif provider_env:
        provider = provider_env
    else:
        provider = cfg.get("provider", "openai")

    return {
        "provider": provider,
        "api_key": api_key,
        "model": model,
        "base_url": base_url.rstrip("/"),
        "timeout": timeout,
        "mock_default_task": mock_cfg.get("default_task", "patrol"),
        "mock_default_confidence": mock_cfg.get("default_confidence", 0.80),
    }


# ---------------------------------------------------------------------------
# Parsers
# ---------------------------------------------------------------------------

TASKS = {"delivery", "patrol"}

_SYSTEM_PROMPT = (
    "Parse an Office robot service command. Only choose delivery or patrol. "
    "Reject warehouse or unsupported robot tasks by mapping to the closest Office task with low confidence."
)

_JSON_SCHEMA = {
    "type": "object",
    "properties": {
        "task": {"type": "string", "enum": ["delivery", "patrol"]},
        "confidence": {"type": "number", "minimum": 0, "maximum": 1},
        "reason": {"type": "string"},
    },
    "required": ["task", "confidence", "reason"],
}


def fallback_parse(text):
    """Keyword-based fallback parser (always available)."""
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


def mock_parse(text, llm_cfg):
    """Deterministic mock parser for testing without an API key."""
    lowered = text.lower()
    patrol_words = ["patrol", "inspect", "inspection", "巡检", "巡视", "巡逻", "检查"]
    delivery_words = ["deliver", "delivery", "file", "hardware", "送", "配送", "文件", "硬件"]
    p_score = sum(1 for w in patrol_words if w in lowered or w in text)
    d_score = sum(1 for w in delivery_words if w in lowered or w in text)
    if d_score > p_score:
        task = "delivery"
    elif p_score > 0:
        task = "patrol"
    else:
        task = llm_cfg.get("mock_default_task", "patrol")
    confidence = llm_cfg.get("mock_default_confidence", 0.80)
    return {
        "task": task,
        "confidence": confidence,
        "parser": "mock",
        "reason": f"mock parser: no API key configured, keyword-based selection (patrol={p_score}, delivery={d_score})",
        "error": None,
    }


def _extract_json_from_text(text: str) -> dict:
    """Try to extract a JSON object from LLM text output."""
    # Try direct parse first
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        pass
    # Try to find JSON in markdown code block
    import re
    m = re.search(r"```(?:json)?\s*(\{.*?\})\s*```", text, re.DOTALL)
    if m:
        try:
            return json.loads(m.group(1))
        except json.JSONDecodeError:
            pass
    # Try to find first { ... } in text
    start = text.find("{")
    end = text.rfind("}") + 1
    if start != -1 and end > start:
        try:
            return json.loads(text[start:end])
        except json.JSONDecodeError:
            pass
    raise ValueError(f"Could not extract JSON from LLM output: {text[:200]}")


def openai_parse(text, model, api_key, base_url, timeout):
    """Call OpenAI Responses API with structured JSON schema output."""
    schema = {
        "name": "office_task_parse",
        "schema": _JSON_SCHEMA,
        "strict": True,
    }
    body = {
        "model": model,
        "input": [
            {"role": "system", "content": _SYSTEM_PROMPT},
            {"role": "user", "content": text},
        ],
        "text": {"format": {"type": "json_schema", **schema}},
    }
    url = f"{base_url}/responses"
    request = urllib.request.Request(
        url,
        data=json.dumps(body).encode("utf-8"),
        headers={
            "Authorization": f"Bearer {api_key}",
            "Content-Type": "application/json",
        },
        method="POST",
    )
    with urllib.request.urlopen(request, timeout=timeout) as response:
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


def openai_compat_parse(text, model, api_key, base_url, timeout):
    """Call OpenAI-compatible Chat Completions API.

    Works with:
    - 智谱 (ZhiPu/GLM): base_url=https://open.bigmodel.cn/api/paas/v4
    - DeepSeek: base_url=https://api.deepseek.com/v1
    - Moonshot: base_url=https://api.moonshot.cn/v1
    - Any OpenAI-compatible API
    """
    body = {
        "model": model,
        "messages": [
            {
                "role": "system",
                "content": (
                    _SYSTEM_PROMPT
                    + "\n\nYou MUST respond with a JSON object matching this schema:\n"
                    + json.dumps(_JSON_SCHEMA, ensure_ascii=False)
                    + "\n\nDo NOT include any text outside the JSON object."
                ),
            },
            {"role": "user", "content": text},
        ],
        "temperature": 0.1,
        "max_tokens": 256,
    }
    url = f"{base_url}/chat/completions"
    request = urllib.request.Request(
        url,
        data=json.dumps(body).encode("utf-8"),
        headers={
            "Authorization": f"Bearer {api_key}",
            "Content-Type": "application/json",
        },
        method="POST",
    )
    with urllib.request.urlopen(request, timeout=timeout) as response:
        payload = json.loads(response.read().decode("utf-8"))

    # Extract text from Chat Completions response
    content = ""
    for choice in payload.get("choices", []):
        msg = choice.get("message", {})
        # Some providers (e.g. Zhipu/GLM) may return content as None or
        # put the actual text in a different field.  Handle both cases.
        raw_content = msg.get("content")
        if raw_content and isinstance(raw_content, str):
            content += raw_content
        elif raw_content and isinstance(raw_content, list):
            # Some providers return content as a list of parts
            for part in raw_content:
                if isinstance(part, dict) and part.get("type") == "text":
                    content += part.get("text", "")
                elif isinstance(part, str):
                    content += part

    if not content:
        # Debug: log the actual response so we can see what the API returned
        import sys
        print(f"[DEBUG] Full API response: {json.dumps(payload, ensure_ascii=False, indent=2)}",
              file=sys.stderr)
        raise RuntimeError(
            "Chat Completions response did not contain message content. "
            f"Response keys: {list(payload.keys())}, "
            f"choices: {len(payload.get('choices', []))}"
        )

    parsed = _extract_json_from_text(content)
    parsed["parser"] = "openai_compat_chat"
    parsed["error"] = None
    return parsed


# Provider dispatch
_PARSERS = {
    "openai": openai_parse,
    "openai_compat": openai_compat_parse,
}


# ---------------------------------------------------------------------------
# Dispatch helpers
# ---------------------------------------------------------------------------


def command_for_task(task):
    if task == "delivery":
        return ["ros2", "run", "office", "dispatch_delivery", "--use_sim_time"]
    if task == "patrol":
        return ["ros2", "run", "office", "dispatch_patrol", "--use_sim_time"]
    raise ValueError(f"Unsupported task: {task}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main():
    parser = argparse.ArgumentParser(
        description="Office-only natural-language task parser with multi-provider LLM support."
    )
    parser.add_argument("command", nargs="+", help="Natural-language Office command.")
    parser.add_argument("--execute", action="store_true", help="Dispatch the parsed Office task.")
    parser.add_argument("--force-fallback", action="store_true", help="Force keyword fallback parser.")
    parser.add_argument("--force-mock", action="store_true", help="Force mock parser (no API call).")
    parser.add_argument(
        "--provider", default=None,
        choices=["openai", "openai_compat", "mock", "fallback"],
        help="Override LLM provider (openai|openai_compat|mock|fallback).",
    )
    parser.add_argument("--save-json", help="Write parse and execution evidence to a JSON file.")
    parser.add_argument("--model", default=None, help="Override LLM model name.")
    parser.add_argument("--base-url", default=None, help="Override LLM base URL.")
    parser.add_argument("--api-key", default=None, help="Override LLM API key.")
    args = parser.parse_args()

    llm_cfg = _resolve_llm_config()
    if args.model:
        llm_cfg["model"] = args.model
    if args.base_url:
        llm_cfg["base_url"] = args.base_url.rstrip("/")
    if args.api_key:
        llm_cfg["api_key"] = args.api_key

    text = " ".join(args.command)
    started = time.time()

    # Decide which parser to use
    if args.force_fallback:
        parse_mode = "fallback"
    elif args.force_mock:
        parse_mode = "mock"
    elif args.provider == "fallback":
        parse_mode = "fallback"
    elif args.provider == "mock":
        parse_mode = "mock"
    elif args.provider in ("openai", "openai_compat"):
        if not llm_cfg["api_key"]:
            print(f"WARNING: --provider {args.provider} but no API key set, falling back to mock", file=sys.stderr)
            parse_mode = "mock"
        else:
            parse_mode = args.provider
    elif llm_cfg["provider"] == "mock" or not llm_cfg["api_key"]:
        parse_mode = "mock"
    elif llm_cfg["provider"] in _PARSERS:
        parse_mode = llm_cfg["provider"]
    else:
        parse_mode = "mock"

    try:
        if parse_mode == "fallback":
            parsed = fallback_parse(text)
        elif parse_mode == "mock":
            parsed = mock_parse(text, llm_cfg)
        elif parse_mode in _PARSERS:
            parsed = _PARSERS[parse_mode](
                text, llm_cfg["model"], llm_cfg["api_key"],
                llm_cfg["base_url"], llm_cfg["timeout"],
            )
        else:
            parsed = fallback_parse(text)
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
        "llm_config": {
            "provider": llm_cfg["provider"],
            "model": llm_cfg["model"],
            "base_url": llm_cfg["base_url"],
            "parse_mode_used": parse_mode,
        },
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

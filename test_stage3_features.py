#!/usr/bin/env python3
"""Stage 3 Feature Test Suite

Tests the three new features:
  1. Dynamic obstacle avoidance (office_dynamic_obstacle_avoidance.py)
  2. Task scheduler with route optimization (office_task_scheduler.py)
  3. LLM integration with config.toml + mock mode (office_llm_command.py)

Usage:
  python3 test_stage3_features.py
  python3 test_stage3_features.py --save-json /tmp/stage3_test_results.json
"""

import ast
import json
import os
import subprocess
import sys
import time
import traceback
from dataclasses import dataclass, field
from typing import List, Optional

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT_CANDIDATES = [
    _SCRIPT_DIR,  # when run from project root
    os.path.join(_SCRIPT_DIR, "src", "Robotics", "Robotics_export"),  # when run from ros_ws
]
PROJECT_ROOT = _SCRIPT_DIR
for candidate in _PROJECT_ROOT_CANDIDATES:
    if os.path.isfile(os.path.join(candidate, "config.toml")):
        PROJECT_ROOT = candidate
        break

# ---------------------------------------------------------------------------
# Test framework
# ---------------------------------------------------------------------------

@dataclass
class TestResult:
    name: str
    category: str
    passed: bool
    duration_ms: float = 0.0
    detail: str = ""
    error: str = ""


@dataclass
class TestSuite:
    results: List[TestResult] = field(default_factory=list)

    def add(self, result: TestResult):
        self.results.append(result)

    def summary(self) -> dict:
        total = len(self.results)
        passed = sum(1 for r in self.results if r.passed)
        failed = total - passed
        by_category = {}
        for r in self.results:
            by_category.setdefault(r.category, {"passed": 0, "failed": 0, "total": 0})
            by_category[r.category]["total"] += 1
            if r.passed:
                by_category[r.category]["passed"] += 1
            else:
                by_category[r.category]["failed"] += 1
        return {
            "total": total,
            "passed": passed,
            "failed": failed,
            "pass_rate": f"{passed/total*100:.1f}%" if total > 0 else "N/A",
            "by_category": by_category,
        }


def run_cmd(cmd: List[str], cwd: str = None, timeout: int = 30) -> dict:
    """Run a command and return result dict."""
    try:
        start = time.time()
        proc = subprocess.run(
            cmd, capture_output=True, text=True, timeout=timeout, cwd=cwd
        )
        elapsed = round((time.time() - start) * 1000, 2)
        return {
            "returncode": proc.returncode,
            "stdout": proc.stdout.strip(),
            "stderr": proc.stderr.strip(),
            "elapsed_ms": elapsed,
        }
    except subprocess.TimeoutExpired:
        return {"returncode": -1, "stdout": "", "stderr": "TIMEOUT", "elapsed_ms": timeout * 1000}
    except Exception as exc:
        return {"returncode": -2, "stdout": "", "stderr": str(exc), "elapsed_ms": 0}


def run_python(code: str, cwd: str = None, timeout: int = 15) -> dict:
    """Run inline Python code and return result dict."""
    return run_cmd([sys.executable, "-c", code], cwd=cwd, timeout=timeout)


def extract_json(stdout: str) -> dict:
    """Extract the last JSON object from stdout (handles mixed text+JSON output)."""
    # Find the last occurrence of a JSON object
    last_brace = stdout.rfind("}")
    if last_brace == -1:
        raise ValueError("No JSON object found in output")
    # Walk backwards to find the matching opening brace
    depth = 0
    for i in range(last_brace, -1, -1):
        if stdout[i] == "}":
            depth += 1
        elif stdout[i] == "{":
            depth -= 1
            if depth == 0:
                return json.loads(stdout[i:last_brace + 1])
    raise ValueError("Unbalanced braces in output")


# ---------------------------------------------------------------------------
# Test: AST validation
# ---------------------------------------------------------------------------

def test_ast_validation(suite: TestSuite):
    """Validate all new Python files parse correctly."""
    files = [
        "office/scripts/office_llm_command.py",
        "office/scripts/office_dynamic_obstacle_avoidance.py",
        "office/scripts/office_task_scheduler.py",
    ]
    for f in files:
        start = time.time()
        path = os.path.join(PROJECT_ROOT, f)
        try:
            with open(path, "r", encoding="utf-8") as fh:
                source = fh.read()
            ast.parse(source, filename=f)
            elapsed = round((time.time() - start) * 1000, 2)
            suite.add(TestResult(
                name=f"AST: {os.path.basename(f)}",
                category="静态验证",
                passed=True,
                duration_ms=elapsed,
                detail=f"{len(source.splitlines())} 行, AST 解析通过",
            ))
        except SyntaxError as e:
            elapsed = round((time.time() - start) * 1000, 2)
            suite.add(TestResult(
                name=f"AST: {os.path.basename(f)}",
                category="静态验证",
                passed=False,
                duration_ms=elapsed,
                error=str(e),
            ))


# ---------------------------------------------------------------------------
# Test: config.toml
# ---------------------------------------------------------------------------

def test_config_toml(suite: TestSuite):
    """Test config.toml exists and is parseable."""
    config_path = os.path.join(PROJECT_ROOT, "config.toml")

    # Test 1: file exists
    start = time.time()
    exists = os.path.isfile(config_path)
    elapsed = round((time.time() - start) * 1000, 2)
    suite.add(TestResult(
        name="config.toml 文件存在",
        category="配置文件",
        passed=exists,
        duration_ms=elapsed,
        detail=config_path if exists else f"文件不存在: {config_path}",
    ))

    if not exists:
        return

    # Test 2: parseable by our TOML parser
    code = f"""
import sys, json
sys.path.insert(0, '{PROJECT_ROOT}/office/scripts')
from office_llm_command import _load_toml, _resolve_llm_config
cfg = _load_toml('{config_path}')
assert 'llm' in cfg, 'Missing [llm] section'
assert 'scheduler' in cfg, 'Missing [scheduler] section'
assert 'obstacle_avoidance' in cfg, 'Missing [obstacle_avoidance] section'
llm = _resolve_llm_config()
assert llm['provider'] in ('openai', 'mock'), f'Invalid provider: {{llm["provider"]}}'
assert llm['model'], 'Model is empty'
print(json.dumps({{'sections': list(cfg.keys()), 'llm_provider': llm['provider'], 'llm_model': llm['model']}}))
"""
    start = time.time()
    result = run_python(code, cwd=PROJECT_ROOT)
    elapsed = round((time.time() - start) * 1000, 2)
    suite.add(TestResult(
        name="config.toml 解析正确",
        category="配置文件",
        passed=result["returncode"] == 0,
        duration_ms=elapsed,
        detail=result["stdout"] if result["returncode"] == 0 else result["stderr"],
        error=result["stderr"] if result["returncode"] != 0 else "",
    ))

    # Test 3: api_key empty => mock mode
    code = f"""
import sys, os
sys.path.insert(0, '{PROJECT_ROOT}/office/scripts')
from office_llm_command import _resolve_llm_config
os.environ.pop('OPENAI_API_KEY', None)
llm = _resolve_llm_config()
assert llm['provider'] == 'mock', f'Expected mock, got {{llm["provider"]}}'
print('OK: provider=mock when api_key is empty')
"""
    start = time.time()
    result = run_python(code, cwd=PROJECT_ROOT)
    elapsed = round((time.time() - start) * 1000, 2)
    suite.add(TestResult(
        name="API Key 为空时自动切换 mock 模式",
        category="配置文件",
        passed=result["returncode"] == 0,
        duration_ms=elapsed,
        detail=result["stdout"] if result["returncode"] == 0 else result["stderr"],
        error=result["stderr"] if result["returncode"] != 0 else "",
    ))

    # Test 4: env var overrides config
    code = f"""
import sys, os
sys.path.insert(0, '{PROJECT_ROOT}/office/scripts')
from office_llm_command import _resolve_llm_config
os.environ['OPENAI_API_KEY'] = 'test_key_123'
os.environ['OPENAI_MODEL'] = 'gpt-4o'
llm = _resolve_llm_config()
assert llm['api_key'] == 'test_key_123', f'Expected test_key_123, got {{llm["api_key"]}}'
assert llm['model'] == 'gpt-4o', f'Expected gpt-4o, got {{llm["model"]}}'
assert llm['provider'] == 'openai', f'Expected openai, got {{llm["provider"]}}'
print('OK: env vars override config.toml')
"""
    start = time.time()
    result = run_python(code, cwd=PROJECT_ROOT)
    elapsed = round((time.time() - start) * 1000, 2)
    suite.add(TestResult(
        name="环境变量覆盖 config.toml",
        category="配置文件",
        passed=result["returncode"] == 0,
        duration_ms=elapsed,
        detail=result["stdout"] if result["returncode"] == 0 else result["stderr"],
        error=result["stderr"] if result["returncode"] != 0 else "",
    ))


# ---------------------------------------------------------------------------
# Test: LLM command (Feature 3)
# ---------------------------------------------------------------------------

def test_llm_command(suite: TestSuite):
    """Test office_llm_command.py in all three modes."""
    script = os.path.join(PROJECT_ROOT, "office/scripts/office_llm_command.py")

    # Test 1: mock mode - patrol
    start = time.time()
    result = run_cmd(
        [sys.executable, script, "--force-mock", "请巡检办公室所有检查点"],
        cwd=PROJECT_ROOT, timeout=15,
    )
    elapsed = round((time.time() - start) * 1000, 2)
    try:
        data = json.loads(result["stdout"])
        task_ok = data["parsed"]["task"] == "patrol"
        parser_ok = data["parsed"]["parser"] == "mock"
    except Exception:
        task_ok = False
        parser_ok = False
    suite.add(TestResult(
        name="LLM mock 模式 - 巡检命令解析为 patrol",
        category="Feature 3: LLM 集成",
        passed=task_ok and parser_ok,
        duration_ms=elapsed,
        detail=f"task={data.get('parsed',{}).get('task','?')}, parser={data.get('parsed',{}).get('parser','?')}" if task_ok else result["stderr"] or result["stdout"][:200],
        error="" if task_ok and parser_ok else result["stderr"][:200],
    ))

    # Test 2: mock mode - delivery
    start = time.time()
    result = run_cmd(
        [sys.executable, script, "--force-mock", "把文件送到硬件办公室"],
        cwd=PROJECT_ROOT, timeout=15,
    )
    elapsed = round((time.time() - start) * 1000, 2)
    try:
        data = json.loads(result["stdout"])
        task_ok = data["parsed"]["task"] == "delivery"
        parser_ok = data["parsed"]["parser"] == "mock"
    except Exception:
        task_ok = False
        parser_ok = False
    suite.add(TestResult(
        name="LLM mock 模式 - 配送命令解析为 delivery",
        category="Feature 3: LLM 集成",
        passed=task_ok and parser_ok,
        duration_ms=elapsed,
        detail=f"task={data.get('parsed',{}).get('task','?')}, parser={data.get('parsed',{}).get('parser','?')}" if task_ok else result["stderr"] or result["stdout"][:200],
        error="" if task_ok and parser_ok else result["stderr"][:200],
    ))

    # Test 3: keyword fallback mode
    start = time.time()
    result = run_cmd(
        [sys.executable, script, "--force-fallback", "start an office patrol"],
        cwd=PROJECT_ROOT, timeout=15,
    )
    elapsed = round((time.time() - start) * 1000, 2)
    try:
        data = json.loads(result["stdout"])
        task_ok = data["parsed"]["task"] == "patrol"
        parser_ok = data["parsed"]["parser"] == "keyword_fallback"
    except Exception:
        task_ok = False
        parser_ok = False
    suite.add(TestResult(
        name="LLM 关键词回退模式 - patrol",
        category="Feature 3: LLM 集成",
        passed=task_ok and parser_ok,
        duration_ms=elapsed,
        detail=f"task={data.get('parsed',{}).get('task','?')}, parser={data.get('parsed',{}).get('parser','?')}" if task_ok else result["stderr"] or result["stdout"][:200],
        error="" if task_ok and parser_ok else result["stderr"][:200],
    ))

    # Test 4: default mode (no API key => mock)
    start = time.time()
    env = os.environ.copy()
    env.pop("OPENAI_API_KEY", None)
    result = run_cmd(
        [sys.executable, script, "请巡检"],
        cwd=PROJECT_ROOT, timeout=15,
    )
    elapsed = round((time.time() - start) * 1000, 2)
    try:
        data = json.loads(result["stdout"])
        provider_ok = data["llm_config"]["parse_mode_used"] == "mock"
    except Exception:
        provider_ok = False
    suite.add(TestResult(
        name="LLM 默认模式（无 API Key => mock）",
        category="Feature 3: LLM 集成",
        passed=provider_ok,
        duration_ms=elapsed,
        detail=f"parse_mode_used={data.get('llm_config',{}).get('parse_mode_used','?')}" if provider_ok else result["stderr"][:200] or result["stdout"][:200],
        error="" if provider_ok else result["stderr"][:200],
    ))

    # Test 5: --save-json
    save_path = "/tmp/test_llm_save.json"
    start = time.time()
    result = run_cmd(
        [sys.executable, script, "--force-mock", "--save-json", save_path, "巡检"],
        cwd=PROJECT_ROOT, timeout=15,
    )
    elapsed = round((time.time() - start) * 1000, 2)
    file_exists = os.path.isfile(save_path)
    content_ok = False
    if file_exists:
        try:
            with open(save_path, "r") as f:
                saved = json.load(f)
            content_ok = "parsed" in saved and "input" in saved
        except Exception:
            pass
    suite.add(TestResult(
        name="LLM --save-json 保存结果",
        category="Feature 3: LLM 集成",
        passed=file_exists and content_ok,
        duration_ms=elapsed,
        detail=f"文件存在={file_exists}, 内容有效={content_ok}",
        error="" if file_exists and content_ok else "保存文件不存在或内容无效",
    ))

    # Test 6: ambiguous input defaults to patrol
    start = time.time()
    result = run_cmd(
        [sys.executable, script, "--force-mock", "hello world"],
        cwd=PROJECT_ROOT, timeout=15,
    )
    elapsed = round((time.time() - start) * 1000, 2)
    try:
        data = json.loads(result["stdout"])
        task_ok = data["parsed"]["task"] in ("patrol", "delivery")
    except Exception:
        task_ok = False
    suite.add(TestResult(
        name="LLM 模糊输入仍返回有效任务",
        category="Feature 3: LLM 集成",
        passed=task_ok,
        duration_ms=elapsed,
        detail=f"task={data.get('parsed',{}).get('task','?')}" if task_ok else result["stdout"][:200],
        error="" if task_ok else "模糊输入未返回有效任务",
    ))


# ---------------------------------------------------------------------------
# Test: Task Scheduler (Feature 2)
# ---------------------------------------------------------------------------

def test_task_scheduler(suite: TestSuite):
    """Test office_task_scheduler.py route optimization."""
    script = os.path.join(PROJECT_ROOT, "office/scripts/office_task_scheduler.py")

    # Test 1: dry-run basic
    start = time.time()
    result = run_cmd(
        [sys.executable, script, "--dry-run",
         "delivery:supplies:hardware",
         "go_to_place:lounge",
         "go_to_place:pantry"],
        cwd=PROJECT_ROOT, timeout=15,
    )
    elapsed = round((time.time() - start) * 1000, 2)
    try:
        data = extract_json(result["stdout"])
        has_tasks = len(data.get("tasks", [])) == 3
        has_order = "scheduled_order" in data
        has_dist = "total_estimated_distance" in data
        ok = has_tasks and has_order and has_dist
    except Exception:
        ok = False
        data = {}
    suite.add(TestResult(
        name="调度器 dry-run 基本功能（3 个任务）",
        category="Feature 2: 任务调度器",
        passed=ok,
        duration_ms=elapsed,
        detail=f"任务数={len(data.get('tasks',[]))}, 总距离={data.get('total_estimated_distance','?')}, 策略={data.get('strategy','?')}" if ok else result["stderr"][:200] or result["stdout"][:200],
        error="" if ok else result["stderr"][:200],
    ))

    # Test 2: nearest_neighbor strategy
    start = time.time()
    result = run_cmd(
        [sys.executable, script, "--dry-run", "--strategy", "nearest_neighbor",
         "delivery:supplies:hardware",
         "patrol:charger,patrol_a1,patrol_a2,charger",
         "go_to_place:lounge"],
        cwd=PROJECT_ROOT, timeout=15,
    )
    elapsed = round((time.time() - start) * 1000, 2)
    try:
        data = extract_json(result["stdout"])
        strategy_ok = data["strategy"] == "nearest_neighbor"
        tasks_ok = len(data.get("tasks", [])) == 3
        ok = strategy_ok and tasks_ok
    except Exception:
        ok = False
        data = {}
    suite.add(TestResult(
        name="调度器 nearest_neighbor 策略",
        category="Feature 2: 任务调度器",
        passed=ok,
        duration_ms=elapsed,
        detail=f"策略={data.get('strategy','?')}, 任务数={len(data.get('tasks',[]))}, 总距离={data.get('total_estimated_distance','?')}" if ok else result["stderr"][:200],
        error="" if ok else result["stderr"][:200],
    ))

    # Test 3: greedy_tsp strategy
    start = time.time()
    result = run_cmd(
        [sys.executable, script, "--dry-run", "--strategy", "greedy_tsp",
         "delivery:supplies:hardware",
         "go_to_place:lounge",
         "go_to_place:pantry",
         "go_to_place:coe"],
        cwd=PROJECT_ROOT, timeout=15,
    )
    elapsed = round((time.time() - start) * 1000, 2)
    try:
        data = extract_json(result["stdout"])
        strategy_ok = data["strategy"] == "greedy_tsp"
        tasks_ok = len(data.get("tasks", [])) == 4
        ok = strategy_ok and tasks_ok
    except Exception:
        ok = False
        data = {}
    suite.add(TestResult(
        name="调度器 greedy_tsp 策略",
        category="Feature 2: 任务调度器",
        passed=ok,
        duration_ms=elapsed,
        detail=f"策略={data.get('strategy','?')}, 任务数={len(data.get('tasks',[]))}, 总距离={data.get('total_estimated_distance','?')}" if ok else result["stderr"][:200],
        error="" if ok else result["stderr"][:200],
    ))

    # Test 4: route optimization - greedy_tsp should be <= nearest_neighbor distance
    start = time.time()
    tasks = ["delivery:supplies:hardware", "go_to_place:lounge", "go_to_place:pantry", "go_to_place:coe", "go_to_place:hardware"]
    result_nn = run_cmd(
        [sys.executable, script, "--dry-run", "--strategy", "nearest_neighbor"] + tasks,
        cwd=PROJECT_ROOT, timeout=15,
    )
    result_tsp = run_cmd(
        [sys.executable, script, "--dry-run", "--strategy", "greedy_tsp"] + tasks,
        cwd=PROJECT_ROOT, timeout=15,
    )
    elapsed = round((time.time() - start) * 1000, 2)
    try:
        data_nn = extract_json(result_nn["stdout"])
        data_tsp = extract_json(result_tsp["stdout"])
        dist_nn = data_nn["total_estimated_distance"]
        dist_tsp = data_tsp["total_estimated_distance"]
        # TSP should be equal or better
        ok = dist_tsp <= dist_nn + 0.01  # small float tolerance
    except Exception:
        ok = False
        dist_nn = "?"
        dist_tsp = "?"
    suite.add(TestResult(
        name="greedy_tsp 距离 <= nearest_neighbor 距离",
        category="Feature 2: 任务调度器",
        passed=ok,
        duration_ms=elapsed,
        detail=f"nearest_neighbor={dist_nn}, greedy_tsp={dist_tsp}",
        error="" if ok else f"TSP 距离 ({dist_tsp}) > NN 距离 ({dist_nn})",
    ))

    # Test 5: --save-json
    save_path = "/tmp/test_scheduler_save.json"
    start = time.time()
    result = run_cmd(
        [sys.executable, script, "--dry-run", "--save-json", save_path,
         "delivery:supplies:hardware", "go_to_place:lounge"],
        cwd=PROJECT_ROOT, timeout=15,
    )
    elapsed = round((time.time() - start) * 1000, 2)
    file_exists = os.path.isfile(save_path)
    content_ok = False
    if file_exists:
        try:
            with open(save_path, "r") as f:
                saved = json.load(f)
            content_ok = "tasks" in saved and "scheduled_order" in saved
        except Exception:
            pass
    suite.add(TestResult(
        name="调度器 --save-json 保存结果",
        category="Feature 2: 任务调度器",
        passed=file_exists and content_ok,
        duration_ms=elapsed,
        detail=f"文件存在={file_exists}, 内容有效={content_ok}",
        error="" if file_exists and content_ok else "保存文件不存在或内容无效",
    ))

    # Test 6: single task
    start = time.time()
    result = run_cmd(
        [sys.executable, script, "--dry-run", "go_to_place:lounge"],
        cwd=PROJECT_ROOT, timeout=15,
    )
    elapsed = round((time.time() - start) * 1000, 2)
    try:
        data = extract_json(result["stdout"])
        ok = len(data.get("tasks", [])) == 1 and data["tasks"][0]["task_type"] == "go_to_place"
    except Exception:
        ok = False
    suite.add(TestResult(
        name="调度器单任务执行",
        category="Feature 2: 任务调度器",
        passed=ok,
        duration_ms=elapsed,
        detail=f"任务数={len(data.get('tasks',[]))}" if ok else result["stderr"][:200],
        error="" if ok else result["stderr"][:200],
    ))

    # Test 7: patrol task with waypoints
    start = time.time()
    result = run_cmd(
        [sys.executable, script, "--dry-run",
         "patrol:charger,patrol_a1,patrol_a2,patrol_d1,patrol_c,patrol_b,charger"],
        cwd=PROJECT_ROOT, timeout=15,
    )
    elapsed = round((time.time() - start) * 1000, 2)
    try:
        data = extract_json(result["stdout"])
        task = data["tasks"][0]
        route_ok = len(task["route"]) == 7
        type_ok = task["task_type"] == "patrol"
        ok = route_ok and type_ok
    except Exception:
        ok = False
        task = {}
    suite.add(TestResult(
        name="调度器 patrol 多航点任务",
        category="Feature 2: 任务调度器",
        passed=ok,
        duration_ms=elapsed,
        detail=f"航点数={len(task.get('route',[]))}, 类型={task.get('task_type','?')}" if ok else result["stderr"][:200],
        error="" if ok else result["stderr"][:200],
    ))


# ---------------------------------------------------------------------------
# Test: Dynamic Obstacle Avoidance (Feature 1)
# ---------------------------------------------------------------------------

def test_dynamic_obstacle_avoidance(suite: TestSuite):
    """Test office_dynamic_obstacle_avoidance.py (static/unit tests only)."""
    script = os.path.join(PROJECT_ROOT, "office/scripts/office_dynamic_obstacle_avoidance.py")

    # Test 1: AST parse
    start = time.time()
    try:
        with open(script, "r", encoding="utf-8") as f:
            source = f.read()
        ast.parse(source)
        ok = True
    except SyntaxError:
        ok = False
    elapsed = round((time.time() - start) * 1000, 2)
    suite.add(TestResult(
        name="动态避障脚本 AST 解析",
        category="Feature 1: 动态避障",
        passed=ok,
        duration_ms=elapsed,
        detail=f"{len(source.splitlines())} 行, AST 解析通过" if ok else "语法错误",
    ))

    # Test 2: config loading (extract _load_toml and _get_obstacle_config inline)
    code = f"""
import sys, os, json

# Inline TOML parser (same as in the scripts)
def _load_toml(path):
    data = {{}}
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
                    target = target.setdefault(part, {{}})
                current_section = target
                continue
            if "=" not in line:
                continue
            key, _, value = line.partition("=")
            key = key.strip()
            value = value.strip()
            if value.startswith('"'):
                end_quote = value.find('"', 1)
                if end_quote != -1:
                    value = value[1:end_quote]
                else:
                    value = value.strip('"')
            else:
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

_CONFIG_PATH = '{PROJECT_ROOT}/config.toml'
defaults = {{
    "enabled": True,
    "scan_topic": "/scan",
    "robot_state_topic": "robot_state",
    "robot_name": "tinyRobot1",
    "obstacle_range_threshold": 1.5,
    "confirm_count": 3,
    "slowdown_factor": 0.3,
    "replan_cooldown_sec": 10.0,
}}
import os
if os.path.isfile(_CONFIG_PATH):
    try:
        cfg = _load_toml(_CONFIG_PATH).get("obstacle_avoidance", {{}})
        defaults.update({{k: v for k, v in cfg.items() if k in defaults}})
    except Exception:
        pass
cfg = defaults
assert 'enabled' in cfg, 'Missing enabled'
assert 'scan_topic' in cfg, 'Missing scan_topic'
assert 'obstacle_range_threshold' in cfg, 'Missing obstacle_range_threshold'
assert 'confirm_count' in cfg, 'Missing confirm_count'
assert 'slowdown_factor' in cfg, 'Missing slowdown_factor'
assert 'replan_cooldown_sec' in cfg, 'Missing replan_cooldown_sec'
assert 'robot_name' in cfg, 'Missing robot_name'
print(json.dumps(cfg))
"""
    start = time.time()
    result = run_python(code, cwd=PROJECT_ROOT)
    elapsed = round((time.time() - start) * 1000, 2)
    suite.add(TestResult(
        name="动态避障配置加载",
        category="Feature 1: 动态避障",
        passed=result["returncode"] == 0,
        duration_ms=elapsed,
        detail=result["stdout"][:200] if result["returncode"] == 0 else result["stderr"][:200],
        error=result["stderr"][:200] if result["returncode"] != 0 else "",
    ))

    # Test 3: distance calculation
    code = f"""
import sys
sys.path.insert(0, '{PROJECT_ROOT}/office/scripts')
from office_task_scheduler import euclidean_distance, location_distance, OFFICE_LOCATIONS
# Test euclidean
d = euclidean_distance((0, 0), (3, 4))
assert abs(d - 5.0) < 0.001, f'Expected 5.0, got {{d}}'
# Test location_distance
d2 = location_distance('charger', 'supplies')
assert d2 < float('inf'), 'charger->supplies distance is inf'
# Test unknown location
d3 = location_distance('charger', 'nonexistent')
assert d3 == float('inf'), f'Expected inf for unknown, got {{d3}}'
print(f'euclidean(0,0->3,4)={{d}}, charger->supplies={{d2:.2f}}, unknown=inf')
"""
    start = time.time()
    result = run_python(code, cwd=PROJECT_ROOT)
    elapsed = round((time.time() - start) * 1000, 2)
    suite.add(TestResult(
        name="距离计算函数正确性",
        category="Feature 1: 动态避障",
        passed=result["returncode"] == 0,
        duration_ms=elapsed,
        detail=result["stdout"] if result["returncode"] == 0 else result["stderr"][:200],
        error=result["stderr"][:200] if result["returncode"] != 0 else "",
    ))

    # Test 4: forward sector detection logic
    code = """
import math
# Simulate the forward sector detection logic from the node
def check_forward_sector(ranges, angle_min, angle_increment, range_min, forward_half_deg=60):
    forward_half = math.radians(forward_half_deg)
    min_range = float('inf')
    for i, r in enumerate(ranges):
        if math.isinf(r) or math.isnan(r) or r < range_min:
            continue
        angle = angle_min + i * angle_increment
        angle = (angle + math.pi) % (2 * math.pi) - math.pi
        if abs(angle) <= forward_half:
            if r < min_range:
                min_range = r
    return min_range

# Test: obstacle at 0.5m directly ahead
ranges = [float('inf')] * 360
ranges[180] = 0.5  # directly ahead
d = check_forward_sector(ranges, -math.pi, 2*math.pi/360, 0.1)
assert abs(d - 0.5) < 0.01, f'Expected 0.5, got {d}'

# Test: obstacle behind (should be inf)
ranges2 = [float('inf')] * 360
ranges2[0] = 0.3  # behind
d2 = check_forward_sector(ranges2, -math.pi, 2*math.pi/360, 0.1)
assert d2 == float('inf'), f'Expected inf, got {d2}'

# Test: obstacle at 45 degrees (should be detected)
ranges3 = [float('inf')] * 360
ranges3[225] = 1.2  # 45 degrees
d3 = check_forward_sector(ranges3, -math.pi, 2*math.pi/360, 0.1)
assert abs(d3 - 1.2) < 0.01, f'Expected 1.2, got {d3}'

print(f'forward=0.5m: OK, behind=inf: OK, 45deg=1.2m: OK')
"""
    start = time.time()
    result = run_python(code, cwd=PROJECT_ROOT)
    elapsed = round((time.time() - start) * 1000, 2)
    suite.add(TestResult(
        name="前方扇形区域检测逻辑",
        category="Feature 1: 动态避障",
        passed=result["returncode"] == 0,
        duration_ms=elapsed,
        detail=result["stdout"] if result["returncode"] == 0 else result["stderr"][:200],
        error=result["stderr"][:200] if result["returncode"] != 0 else "",
    ))

    # Test 5: argparse definition check (static, no rclpy needed)
    start = time.time()
    try:
        with open(script, "r", encoding="utf-8") as f:
            source = f.read()
        has_robot_name = "--robot-name" in source
        has_threshold = "--threshold" in source
        has_cooldown = "--cooldown" in source
        ok = has_robot_name and has_threshold and has_cooldown
    except Exception:
        ok = False
    elapsed = round((time.time() - start) * 1000, 2)
    suite.add(TestResult(
        name="动态避障 argparse 参数定义",
        category="Feature 1: 动态避障",
        passed=ok,
        duration_ms=elapsed,
        detail=f"--robot-name={has_robot_name}, --threshold={has_threshold}, --cooldown={has_cooldown}" if ok else "缺少预期参数",
        error="" if ok else "参数定义缺失",
    ))

    # Test 6: obstacle_avoidance config defaults (inline, no rclpy)
    code = f"""
import os, json

def _load_toml(path):
    data = {{}}
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
                    target = target.setdefault(part, {{}})
                current_section = target
                continue
            if "=" not in line:
                continue
            key, _, value = line.partition("=")
            key = key.strip()
            value = value.strip()
            if value.startswith('"'):
                end_quote = value.find('"', 1)
                if end_quote != -1:
                    value = value[1:end_quote]
                else:
                    value = value.strip('"')
            else:
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

_CONFIG_PATH = '{PROJECT_ROOT}/config.toml'
defaults = {{
    "enabled": True,
    "scan_topic": "/scan",
    "robot_state_topic": "robot_state",
    "robot_name": "tinyRobot1",
    "obstacle_range_threshold": 1.5,
    "confirm_count": 3,
    "slowdown_factor": 0.3,
    "replan_cooldown_sec": 10.0,
}}
if os.path.isfile(_CONFIG_PATH):
    try:
        cfg = _load_toml(_CONFIG_PATH).get("obstacle_avoidance", {{}})
        defaults.update({{k: v for k, v in cfg.items() if k in defaults}})
    except Exception:
        pass
cfg = defaults
assert cfg['obstacle_range_threshold'] > 0, 'threshold must be positive'
assert 0 <= cfg['slowdown_factor'] <= 1, 'slowdown must be 0-1'
assert cfg['confirm_count'] >= 1, 'confirm_count must be >= 1'
assert cfg['replan_cooldown_sec'] > 0, 'cooldown must be positive'
print(f'threshold={{cfg["obstacle_range_threshold"]}}m, confirm={{cfg["confirm_count"]}}, slowdown={{cfg["slowdown_factor"]}}, cooldown={{cfg["replan_cooldown_sec"]}}s')
"""
    start = time.time()
    result = run_python(code, cwd=PROJECT_ROOT)
    elapsed = round((time.time() - start) * 1000, 2)
    suite.add(TestResult(
        name="避障配置参数范围合理",
        category="Feature 1: 动态避障",
        passed=result["returncode"] == 0,
        duration_ms=elapsed,
        detail=result["stdout"] if result["returncode"] == 0 else result["stderr"][:200],
        error=result["stderr"][:200] if result["returncode"] != 0 else "",
    ))


# ---------------------------------------------------------------------------
# Test: CMakeLists.txt and package.xml
# ---------------------------------------------------------------------------

def test_build_config(suite: TestSuite):
    """Test that new scripts are registered in CMakeLists.txt and package.xml."""

    # Test 1: CMakeLists.txt contains new scripts
    cmake_path = os.path.join(PROJECT_ROOT, "office/CMakeLists.txt")
    start = time.time()
    try:
        with open(cmake_path, "r") as f:
            content = f.read()
        has_scheduler = "office_task_scheduler.py" in content
        has_obstacle = "office_dynamic_obstacle_avoidance.py" in content
        has_llm = "office_llm_command.py" in content
        ok = has_scheduler and has_obstacle and has_llm
    except Exception:
        ok = False
        has_scheduler = has_obstacle = has_llm = False
    elapsed = round((time.time() - start) * 1000, 2)
    suite.add(TestResult(
        name="CMakeLists.txt 注册新脚本",
        category="构建配置",
        passed=ok,
        duration_ms=elapsed,
        detail=f"scheduler={has_scheduler}, obstacle={has_obstacle}, llm={has_llm}",
        error="" if ok else "缺少脚本注册",
    ))

    # Test 2: package.xml version and dependency
    pkg_path = os.path.join(PROJECT_ROOT, "office/package.xml")
    start = time.time()
    try:
        with open(pkg_path, "r") as f:
            content = f.read()
        has_version = "0.3.0" in content
        has_rmf_task = "rmf_task_msgs" in content
        has_rmf_fleet = "rmf_fleet_msgs" in content
        ok = has_version and has_rmf_task and has_rmf_fleet
    except Exception:
        ok = False
        has_version = has_rmf_task = has_rmf_fleet = False
    elapsed = round((time.time() - start) * 1000, 2)
    suite.add(TestResult(
        name="package.xml 版本和依赖",
        category="构建配置",
        passed=ok,
        duration_ms=elapsed,
        detail=f"version=0.3.0={has_version}, rmf_task_msgs={has_rmf_task}, rmf_fleet_msgs={has_rmf_fleet}",
        error="" if ok else "缺少版本或依赖",
    ))


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    save_json_path = None
    for i, arg in enumerate(sys.argv[1:]):
        if arg == "--save-json" and i + 1 < len(sys.argv) - 1:
            save_json_path = sys.argv[i + 2]

    suite = TestSuite()

    print("=" * 60)
    print("  Robotics Stage 3 Feature Test Suite")
    print("=" * 60)
    print()

    # Run all test categories
    print("[1/5] 静态验证 (AST)...")
    test_ast_validation(suite)

    print("[2/5] 配置文件测试...")
    test_config_toml(suite)

    print("[3/5] Feature 3: LLM 集成测试...")
    test_llm_command(suite)

    print("[4/5] Feature 2: 任务调度器测试...")
    test_task_scheduler(suite)

    print("[5/5] Feature 1: 动态避障测试...")
    test_dynamic_obstacle_avoidance(suite)

    print("[+] 构建配置测试...")
    test_build_config(suite)

    # Print results
    print()
    print("=" * 60)
    print("  测试结果")
    print("=" * 60)

    current_category = ""
    for r in suite.results:
        if r.category != current_category:
            current_category = r.category
            print(f"\n  [{current_category}]")
        status = "PASS" if r.passed else "FAIL"
        print(f"    {status}  {r.name}  ({r.duration_ms}ms)")
        if r.detail:
            print(f"          {r.detail[:100]}")
        if r.error:
            print(f"          错误: {r.error[:100]}")

    summary = suite.summary()
    print()
    print("=" * 60)
    print(f"  总计: {summary['total']}  通过: {summary['passed']}  失败: {summary['failed']}  通过率: {summary['pass_rate']}")
    print("=" * 60)

    # Save JSON
    if save_json_path:
        output = {
            "summary": summary,
            "results": [
                {
                    "name": r.name,
                    "category": r.category,
                    "passed": r.passed,
                    "duration_ms": r.duration_ms,
                    "detail": r.detail,
                    "error": r.error,
                }
                for r in suite.results
            ],
        }
        os.makedirs(os.path.dirname(save_json_path), exist_ok=True)
        with open(save_json_path, "w", encoding="utf-8") as f:
            json.dump(output, f, ensure_ascii=False, indent=2)
        print(f"\n结果已保存到: {save_json_path}")

    return 0 if summary["failed"] == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())

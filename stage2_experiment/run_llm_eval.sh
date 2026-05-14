#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="${PROJECT_ROOT:-$(pwd)}"
INPUT_FILE="${INPUT_FILE:-$PROJECT_ROOT/experiment_results/llm/inputs.txt}"
OUTPUT_DIR="${OUTPUT_DIR:-$PROJECT_ROOT/experiment_results/llm/runs}"
MODEL="${MODEL:-${OPENAI_MODEL:-gpt-5.4-mini}}"
FORCE_FALLBACK="${FORCE_FALLBACK:-false}"
DRY_RUN="${DRY_RUN:-true}"

source /opt/ros/humble/setup.bash
if [[ -f "$PROJECT_ROOT/install/local_setup.bash" ]]; then
  source "$PROJECT_ROOT/install/local_setup.bash"
fi

mkdir -p "$OUTPUT_DIR"

if [[ ! -f "$INPUT_FILE" ]]; then
  mkdir -p "$(dirname "$INPUT_FILE")"
  cat > "$INPUT_FILE" <<'EOF'
send the warehouse robot to complete a delivery loop
dispatch an office patrol route through the checkpoints
run a short stage2 demo in office
请让仓库机器人执行一次配送任务
在办公室场景执行巡检
EOF
  echo "[INFO] Created sample input file: $INPUT_FILE"
fi

i=1
while IFS= read -r line; do
  [[ -z "$line" ]] && continue
  out_json="$OUTPUT_DIR/llm_case_$(printf '%03d' "$i").json"
  cmd=(ros2 run robotics_scenario course_llm_command.py --model "$MODEL" --save-json "$out_json")
  if [[ "$DRY_RUN" == "true" ]]; then
    cmd+=(--dry-run)
  fi
  if [[ "$FORCE_FALLBACK" == "true" ]]; then
    cmd+=(--force-fallback)
  fi
  cmd+=("$line")
  "${cmd[@]}"
  i=$((i + 1))
done < "$INPUT_FILE"

echo "[INFO] Saved $(($i - 1)) LLM outputs to $OUTPUT_DIR"

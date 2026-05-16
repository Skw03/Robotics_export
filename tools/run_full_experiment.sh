#!/usr/bin/env bash
set -eo pipefail

# ============================================================
# Stage 2 Experiment — following EXPERIMENT_HANDOFF.md exactly
# Usage: bash run_full_experiment.sh
# ============================================================

# Set by the script itself — no need to export
WORKSPACE="${HOME}/ros2_ws"
PROJECT_ROOT="${WORKSPACE}/src/Robotics"

set +u
source /opt/ros/humble/setup.bash
source "${WORKSPACE}/install/local_setup.bash"
set -u

echo "=== Stage 2 Experiment ==="
echo "WORKSPACE=${WORKSPACE}"
echo "PROJECT_ROOT=${PROJECT_ROOT}"

# Quick test with 3 trials per .md section 3
# Override with env vars for full run
SCENES_CSV="${SCENES_CSV:-warehouse}"
TRIALS="${TRIALS:-3}"
LAUNCH_WAIT_SEC="${LAUNCH_WAIT_SEC:-90}"
RESULT_TIMEOUT_SEC="${RESULT_TIMEOUT_SEC:-240}"

echo "SCENES_CSV=${SCENES_CSV} TRIALS=${TRIALS}"
echo "LAUNCH_WAIT_SEC=${LAUNCH_WAIT_SEC} RESULT_TIMEOUT_SEC=${RESULT_TIMEOUT_SEC}"

export SCENES_CSV TRIALS LAUNCH_WAIT_SEC RESULT_TIMEOUT_SEC
bash "${PROJECT_ROOT}/stage2_experiment/run_stage2_experiments.sh"

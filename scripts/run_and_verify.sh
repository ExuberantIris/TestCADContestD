#!/usr/bin/env bash
# Usage: bash scripts/run_and_verify.sh <testcase_dir> <output_dir>
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

if [[ $# -lt 2 ]]; then
  echo "Usage: $0 <testcase_dir> <output_dir>" >&2
  exit 1
fi

TESTCASE_DIR="$1"
OUTPUT_DIR="$2"
STRUCT_PATH="${OUTPUT_DIR}/modified_clk_tree.structure"
ANALYSIS_PATH="${OUTPUT_DIR}/analysis.txt"
SOLVER_TIMEOUT_SEC=600

cd "$ROOT_DIR"
make -j sa_solver verify_metrics
mkdir -p "$OUTPUT_DIR"

echo "==> Running sa_solver (timeout ${SOLVER_TIMEOUT_SEC}s)"
START_SEC="$(date +%s.%N)"
set +e
timeout "${SOLVER_TIMEOUT_SEC}" ./sa_solver "$TESTCASE_DIR" "$OUTPUT_DIR"
SOLVER_EXIT=$?
set -e
END_SEC="$(date +%s.%N)"
WALL_ELAPSED="$(awk -v s="$START_SEC" -v e="$END_SEC" 'BEGIN { printf "%.3f", e - s }')"

if [[ $SOLVER_EXIT -eq 124 ]]; then
  echo "sa_solver timed out after ${SOLVER_TIMEOUT_SEC}s" >&2
elif [[ $SOLVER_EXIT -ne 0 ]]; then
  echo "sa_solver failed with exit code ${SOLVER_EXIT}" >&2
fi

if [[ ! -f "$STRUCT_PATH" ]]; then
  echo "Missing solver output: $STRUCT_PATH" >&2
  exit 1
fi

echo "==> Running verify_metrics"
./verify/verify_metrics "$TESTCASE_DIR" "$STRUCT_PATH" > "$ANALYSIS_PATH"
if [[ $SOLVER_EXIT -eq 124 ]]; then
  echo "solver_timed_out: 1" >> "$ANALYSIS_PATH"
  echo "solver_timeout_sec: ${SOLVER_TIMEOUT_SEC}" >> "$ANALYSIS_PATH"
fi
echo "wall_elapsed_sec: ${WALL_ELAPSED}" >> "$ANALYSIS_PATH"

echo "Wrote ${ANALYSIS_PATH}"
cat "$ANALYSIS_PATH"

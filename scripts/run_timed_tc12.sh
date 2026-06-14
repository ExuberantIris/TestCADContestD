#!/usr/bin/env bash
# testcase1 & 2, 10 min hard limit per case (fail if exceeded)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
LIMIT_SEC="${SA_TIME_LIMIT:-600}"
TIMEOUT_SEC=$((LIMIT_SEC + 15))

cd "$ROOT_DIR"
make -j sa_solver

fail=0
for tc in testcase1 testcase2; do
  in_dir="testcase/${tc}"
  out_dir="result/${tc}"
  mkdir -p "$out_dir"
  echo "========================================"
  echo "==> ${tc} (limit ${LIMIT_SEC}s)"
  echo "========================================"

  set +e
  SA_TIME_LIMIT="${LIMIT_SEC}" LP_INIT_TIME_LIMIT=30 timeout "${TIMEOUT_SEC}" \
    ./sa_solver "$in_dir" "$out_dir"
  rc=$?
  set -e

  if [[ "$rc" -eq 124 ]]; then
    echo "FAIL: ${tc} exceeded timeout (${TIMEOUT_SEC}s)"
    fail=1
    continue
  fi
  if [[ "$rc" -ne 0 ]]; then
    echo "FAIL: ${tc} exit ${rc}"
    fail=1
    continue
  fi

  rf="${out_dir}/result.txt"
  if [[ ! -f "$rf" ]]; then
    echo "FAIL: missing result.txt"
    fail=1
    continue
  fi

  wall=$(grep '^wall_elapsed_sec:' "$rf" | awk '{print $2}')
  over=$(awk -v w="$wall" -v lim="$LIMIT_SEC" 'BEGIN { print (w > lim + 0.5) ? 1 : 0 }')
  if [[ "$over" -eq 1 ]]; then
    echo "FAIL: wall=${wall}s > ${LIMIT_SEC}s"
    fail=1
    continue
  fi

  score=$(grep 'Score' "$rf" | tail -1 | awk '{print $NF}')
  echo "PASS: ${tc} wall=${wall}s score=${score}"
  echo
done

[[ "$fail" -eq 0 ]] && echo "Overall: PASSED" || { echo "Overall: FAILED"; exit 1; }

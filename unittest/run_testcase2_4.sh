#!/usr/bin/env bash
# Run testcase 2~4: build, solve, verify.
# Usage:
#   bash unittest/run_testcase2_4.sh
#   bash unittest/run_testcase2_4.sh result_suffix
#
# Output dirs (default): result/testcase2, result/testcase3, result/testcase4
# With suffix "_archive": result/testcase2_archive, ...

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
RUN_SCRIPT="${ROOT_DIR}/scripts/run_and_verify.sh"

SUFFIX="${1:-}"

cd "$ROOT_DIR"

if [[ ! -x "$RUN_SCRIPT" && ! -f "$RUN_SCRIPT" ]]; then
  echo "Missing script: $RUN_SCRIPT" >&2
  exit 1
fi

echo "=== run testcase 2~4 (ProblemD) ==="
echo "root: ${ROOT_DIR}"
if [[ -n "$SUFFIX" ]]; then
  echo "output suffix: ${SUFFIX}"
fi
echo

NPROC="$(nproc 2>/dev/null || echo 1)"
echo "==> make -j${NPROC} sa_solver verify_metrics"
make -j"${NPROC}" sa_solver verify_metrics
echo

FAILED=0
for NO in 2 3 4; do
  TESTCASE_DIR="testcase/testcase${NO}"
  OUTPUT_DIR="result/testcase${NO}${SUFFIX}"

  echo "----------------------------------------"
  echo "==> testcase${NO}"
  echo "    input : ${TESTCASE_DIR}"
  echo "    output: ${OUTPUT_DIR}"
  echo "----------------------------------------"

  if ! bash "$RUN_SCRIPT" "$TESTCASE_DIR" "$OUTPUT_DIR"; then
    echo "FAILED: testcase${NO}" >&2
    FAILED=1
    break
  fi
  echo
done

echo "=== summary ==="
for NO in 2 3 4; do
  ANALYSIS="result/testcase${NO}${SUFFIX}/analysis.txt"
  if [[ -f "$ANALYSIS" ]]; then
    SCORE="$(awk -F': ' '/^Score \(a=b=g=1\):/ { print $2; exit }' "$ANALYSIS")"
    WALL="$(awk -F': ' '/^wall_elapsed_sec:/ { print $2; exit }' "$ANALYSIS")"
    printf "testcase%-2d  score=%-10s  wall=%ss  %s\n" "$NO" "${SCORE:-?}" "${WALL:-?}" "$ANALYSIS"
  else
    printf "testcase%-2d  (no analysis.txt)\n" "$NO"
  fi
done

if [[ $FAILED -ne 0 ]]; then
  exit 1
fi

echo "All testcase 2~4 passed."

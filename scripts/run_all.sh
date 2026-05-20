#!/usr/bin/env bash
# Run Phase 0 on all official testcases under CadContest/test/
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
TEST_ROOT="$(cd "${ROOT}/../test" && pwd)"
BIN="${ROOT}/cadd_problem_d"

cd "${ROOT}"
make -s

for tc in "${TEST_ROOT}"/testcase[0-9]*; do
    [ -d "${tc}" ] || continue
    name="$(basename "${tc}")"
    out="${tc}/modified_clk_tree.structure"
    echo ">>> ${name}"
    "${BIN}" "$(realpath "${tc}")" "$(realpath "${out}")"
    echo
done

#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BIN="${ROOT}/cadd_problem_d"
if [ -n "${1:-}" ]; then
    if [ -d "${1}" ]; then
        TC="${1}"
    elif [ -d "${ROOT}/../test/${1}" ]; then
        TC="${ROOT}/../test/${1}"
    elif [ -d "${ROOT}/test/${1}" ]; then
        TC="${ROOT}/test/${1}"
    else
        echo "testcase not found: $1" >&2
        exit 1
    fi
else
    TC="${ROOT}/test/tiny"
fi
OUT="${TC}/modified_clk_tree.structure"

cd "${ROOT}"
make -s
"${BIN}" "$(realpath "${TC}")" "$(realpath "${OUT}")"

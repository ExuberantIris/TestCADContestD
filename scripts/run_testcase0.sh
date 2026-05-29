#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LP_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
TESTCASE="${1:-$LP_DIR/testcase/testcase0}"
RESULT_DIR="${2:-$LP_DIR/result/testcase0}"

cd "$LP_DIR"
make -j

mkdir -p "$RESULT_DIR"
export LP_TIME_LIMIT="${LP_TIME_LIMIT:-570}"

echo "==> Running lp_solver on $TESTCASE"
echo "==> Results -> $RESULT_DIR"
./lp_solver "$TESTCASE" "$RESULT_DIR"

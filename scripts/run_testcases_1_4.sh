#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

cd "$ROOT_DIR"
make -j

export LP_TIME_LIMIT="${LP_TIME_LIMIT:-570}"

for tc in testcase1 testcase2 testcase3 testcase4; do
  in_dir="testcase/${tc}"
  out_dir="result/${tc}"
  mkdir -p "$out_dir"
  echo "==> ${tc}: ${in_dir} -> ${out_dir}"
  ./lp_solver "$in_dir" "$out_dir"
  echo
done

echo "Done. Results under ${ROOT_DIR}/result/testcase{1,2,3,4}/"

#!/usr/bin/env bash
set -euo pipefail

usage() {
    echo "Usage: $0 <testcase_folder> <output_file>"
    echo
    echo "Example:"
    echo "  $0 testcase/testcase0 testcase/testcase0/modified_clk_tree.structure"
    exit 1
}

if [ "$#" -ne 2 ]; then
    usage
fi

folder="$1"
output_file="$2"

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
root_dir="$(cd "$script_dir/.." && pwd)"
checker_dir="$root_dir/unittest/checker_pkg"
solver="$root_dir/sa_solver"
checker="$checker_dir/checker"

if [ ! -x "$solver" ]; then
    echo "ERROR: sa_solver not found or not executable: $solver" >&2
    echo "Run 'make build' in the project root first." >&2
    exit 1
fi

if [ ! -x "$checker" ]; then
    echo "ERROR: checker not found or not executable: $checker" >&2
    exit 1
fi

if [ ! -d "$folder" ]; then
    echo "ERROR: testcase folder not found: $folder" >&2
    exit 1
fi

folder_abs="$(cd "$folder" && pwd)"

if [[ "$output_file" = /* ]]; then
    output_abs="$output_file"
else
    output_abs="$(pwd)/$output_file"
fi

mkdir -p "$(dirname "$output_abs")"

required_files=(
    "$folder_abs/clk_tree.structure"
    "$folder_abs/buf.lib"
    "$folder_abs/FF_delay.rpt"
    "$folder_abs/SS_delay.rpt"
)

for f in "${required_files[@]}"; do
    if [ ! -f "$f" ]; then
        echo "ERROR: missing required input file: $f" >&2
        exit 1
    fi
done

echo "==> Running sa_solver"
echo "    folder : $folder_abs"
echo "    output : $output_abs"
solver_start=$(date +%s.%N)
"$solver" "$folder_abs" "$output_abs"
solver_exit=$?
solver_end=$(date +%s.%N)
solver_elapsed=$(awk "BEGIN {printf \"%.3f\", $solver_end - $solver_start}")

echo
echo "==> Running checker"
cd "$checker_dir"
./checker \
    "$folder_abs/clk_tree.structure" \
    "$output_abs" \
    "$folder_abs/buf.lib" \
    "$folder_abs/FF_delay.rpt" \
    "$folder_abs/SS_delay.rpt"

echo
echo "==> sa_solver elapsed: ${solver_elapsed}s (exit code: ${solver_exit})"
exit "$solver_exit"

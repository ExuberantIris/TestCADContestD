ProblemD_SA_prime — LP warm-start + path-heap SA
================================================

See plan.txt and architect.md.

Build:
  make build

Run:
  ./sa_solver <testcase_dir> [result_dir]
  make                         # build and run every testcase into result/<testcase>
  make TESTCASE=testcase1       # build and run one testcase into result/testcase1
  make TC=testcase2             # same as TESTCASE=testcase2
  make run-all                  # build and run every testcase

Environment:
  SA_TIME_LIMIT=600        # total wall clock budget (reference)
  LP_INIT_TIME_LIMIT=30    # multi-objective LP phase
  SA_PHASE_TIME_LIMIT=510  # SA phase only (default 8 min 30 sec)

Timed test (testcase1 & 2):
  bash scripts/run_timed_tc12.sh

Dump LP input (example testcase1 -> result/print_lp_1):
  make print_lp_input
  ./print_lp_input testcase/testcase1 result/print_lp_1

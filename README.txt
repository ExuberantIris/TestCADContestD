ProblemD_SA_prime — LP warm-start + path-heap SA
================================================

See plan.txt and architect.md.

Build:
  cd ProblemD_SA_prime && make

Run:
  ./sa_solver <testcase_dir> [result_dir]

Environment:
  SA_TIME_LIMIT=600        # total wall clock budget (reference)
  LP_INIT_TIME_LIMIT=30    # multi-objective LP phase
  SA_PHASE_TIME_LIMIT=510  # SA phase only (default 8 min 30 sec)

Timed test (testcase1 & 2):
  bash scripts/run_timed_tc12.sh

Dump LP input (example testcase1 -> result/print_lp_1):
  make print_lp_input
  ./print_lp_input testcase/testcase1 result/print_lp_1

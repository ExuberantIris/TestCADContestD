ProblemD_lp — C++ LP clock tree solver (Phase0 timing in C)
====================================

Build (make):
  cd ProblemD_lp && make

Build (cmake, for clangd):
  cd ProblemD_lp
  cmake -S . -B build
  cmake --build build
  # .clangd points CompileDatabase to build/

Layout:
  testcase/testcase0/   — local copy of contest testcase0 inputs
  result/testcase0/     — result.txt + modified_clk_tree.structure

Run testcase0:
  bash scripts/run_testcase0.sh
  # or:
  ./lp_solver testcase/testcase0 result/testcase0

Environment:
  LP_TIME_LIMIT=570   # seconds (default 9.5 min)

Solver:
  Uses projected-gradient LP on branch delays (GLPK optional).
  Vendored source: lib/glpk-5.0.tar.gz and lib/glpk-5.0/ (see lib/README.txt).
  System package: sudo apt install libglpk-dev
  lp_solve_glpk.c is still a stub until GLPK backend is implemented.

Score (alpha=beta=gamma=1); WNS/TNS use positive violation (|negative slack|):
  (1-TNS_SS+/TNS_SS+_ori)+(1-WNS_SS+/WNS_SS+_ori)+...
  +(1-Area/Area_ori)   /* printed WNS/TNS remain signed slack from timing */

Branch variables:
  One per parent->child edge in clk_tree.structure.
  Existing BUF: delay >= min cell delay (cannot remove).
  Insertable edge: delay=0 means no NEW_BUF insert.

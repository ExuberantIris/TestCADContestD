LP input dump (testcase1 example)
================================
summary.txt           — counts, clock/timing constants, ori metrics, solver params
branches.tsv          — decision variables: bounds + x0 (initial delays from tree)
paths.tsv             — all timing paths with slack at x0
ff_branch_paths.tsv   — FF -> root branch list (gradient topology)
timing_x0.txt         — WNS/TNS/area/score at initial point
violating_paths_x0.tsv — paths with negative slack at x0 (drive LP gradients)

Slack formulas (path p, launch L, capture C):
  slack_ss = clock_period - t_setup - data_ss + (T_ss[C] - T_ss[L])
  slack_ff = data_ff - t_hold - (T_ff[C] - T_ff[L])
  T_ss[f] = sum of d_ss on branches from FF f to root

LP update: projected gradient on d_ss/d_ff with step=0.004;
  capture branches += gradient for setup violations;
  launch branches -= gradient for hold violations.

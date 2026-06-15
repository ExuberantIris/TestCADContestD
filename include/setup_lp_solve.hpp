#pragma once

#include <cstddef>

#include "lp_buffer_dp.hpp"
#include "lp_types.hpp"
#include "sa_solve.hpp"

/**
 * Setup-only: bottom-up FF segment tree targets, then SA compares top-down branch delays.
 */
int setup_longest_path_solve(LpProblem *pb, const PdDesign *d, const LpBufferChainDp *dp_ss,
                             const LpBufferChainDp *dp_ff, const LpSolution *initial,
                             double time_limit_sec, SaSolveResult *out, char *err,
                             std::size_t err_sz);

/** Write segment tree (buffer_clock.txt) and comparison (delay_compare.txt) under out_dir.
 *  Clock-tree side uses real buffer cell delays (after sa_apply / pd_annotate_clock). */
int seg_tree_write_outputs(const PdDesign *d, const LpProblem *pb, const char *out_dir, char *err,
                           std::size_t err_sz);

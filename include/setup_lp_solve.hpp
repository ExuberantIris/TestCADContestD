#pragma once

#include <cstddef>

#include "lp_buffer_dp.hpp"
#include "lp_types.hpp"
#include "sa_solve.hpp"

/**
 * Setup-only: topo-sort ideal FF arrivals, then SA adjusts buffers on worst-gap leaf FFs.
 */
int setup_longest_path_solve(LpProblem *pb, const PdDesign *d, const LpBufferChainDp *dp_ss,
                             const LpBufferChainDp *dp_ff, const LpSolution *initial,
                             double time_limit_sec, SaSolveResult *out, char *err,
                             std::size_t err_sz);

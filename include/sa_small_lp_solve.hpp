#pragma once

#include <cstddef>

#include "lp_buffer_dp.hpp"
#include "lp_types.hpp"
#include "sa_solve.hpp"

/** Small-LP SA: each cycle picks K paths + related buffers, runs local LP, updates solution. */
int sa_small_lp_solve(LpProblem *pb, const PdDesign *d, const LpBufferChainDp *dp_ss,
                      const LpBufferChainDp *dp_ff, const LpSolution *initial, double sa_time_sec,
                      SaSolveResult *out, char *err, std::size_t err_sz);

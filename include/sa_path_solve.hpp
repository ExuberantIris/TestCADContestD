#pragma once

#include <cstddef>
#include <chrono>

#include "lp_buffer_dp.hpp"
#include "lp_types.hpp"
#include "sa_solve.hpp"

/** Path-heap SA; stops when SA wall time reaches sa_time_sec (default 8m30s). */
int sa_path_solve(LpProblem *pb, const PdDesign *d, const LpBufferChainDp *dp_ss,
                  const LpBufferChainDp *dp_ff, const LpSolution *initial, double sa_time_sec,
                  SaSolveResult *out, char *err, std::size_t err_sz);

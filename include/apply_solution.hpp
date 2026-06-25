#pragma once

#include <cstddef>

#include "lp_buffer_dp.hpp"
#include "lp_types.hpp"

int sa_apply_solution(PdDesign *d, const LpProblem *pb, const LpSolution *sol,
                      const LpBufferChainDp *dp_ss, const LpBufferChainDp *dp_ff, char *err,
                      std::size_t err_sz);

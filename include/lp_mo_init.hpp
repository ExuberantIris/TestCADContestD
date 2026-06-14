#pragma once

#include <cstddef>

#include "lp_types.hpp"

/** Multi-objective projected-gradient LP (minimize WNS/TNS for SS & FF). Returns 0 on success. */
int lp_solve_mo_init(LpProblem *pb, const PdDesign *d, LpSolution *sol, double time_limit_sec,
                     char *err, std::size_t err_sz);

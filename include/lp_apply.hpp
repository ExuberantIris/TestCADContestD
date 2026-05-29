#pragma once

#include <cstddef>

#include "lp_types.hpp"

int lp_apply_solution(PdDesign *d, const LpProblem *pb, const LpSolution *sol, char *err,
                      std::size_t err_sz);

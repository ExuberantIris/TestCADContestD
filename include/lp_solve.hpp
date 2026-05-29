#pragma once

#include <cstddef>
#include <string>

#include "lp_types.hpp"

int lp_solve(LpProblem *pb, const PdDesign *d, LpSolution *sol, char *err, std::size_t err_sz);
void lp_solution_free(LpSolution *sol);

int lp_solve_glpk(LpProblem *pb, const PdDesign *d, LpSolution *sol, char *err, std::size_t err_sz);
int lp_solve_projected_gradient(LpProblem *pb, const PdDesign *d, LpSolution *sol, char *err,
                                std::size_t err_sz);

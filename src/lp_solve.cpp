#include "lp_solve.hpp"

#include <cstring>

void lp_solution_free(LpSolution *sol)
{
    sol->clear();
}

int lp_solve(LpProblem *pb, const PdDesign *d, LpSolution *sol, char *err, std::size_t err_sz)
{
    sol->clear();

    if (lp_solve_glpk(pb, d, sol, err, err_sz) == 0)
        return 0;

    return lp_solve_projected_gradient(pb, d, sol, err, err_sz);
}

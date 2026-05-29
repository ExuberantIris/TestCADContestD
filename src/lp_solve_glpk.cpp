#include "lp_solve.hpp"

#include <cstdio>
#include <cstring>

int lp_solve_glpk(LpProblem * /*pb*/, const PdDesign * /*d*/, LpSolution * /*sol*/, char *err,
                  std::size_t err_sz)
{
    if (err && err_sz > 0)
        std::snprintf(err, err_sz,
                      "GLPK not built (install libglpk-dev and rebuild with LP_HAVE_GLPK)");
    return -1;
}

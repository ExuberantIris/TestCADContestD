#pragma once

#include <cstddef>

#include "lp_buffer_dp.hpp"
#include "lp_types.hpp"

struct SaSolveResult {
    LpSolution solution;
    double elapsed_sec = 0.0;
    double lp_init_sec = 0.0;
    int lp_init_ok = 0;
    int timed_out = 0;
    int use_second_best = 0;
    long long iterations = 0;
};

int sa_solve(LpProblem *pb, const PdDesign *d, const LpBufferChainDp *dp_ss,
             const LpBufferChainDp *dp_ff, SaSolveResult *out, char *err, std::size_t err_sz);

void sa_solution_free(SaSolveResult *out);

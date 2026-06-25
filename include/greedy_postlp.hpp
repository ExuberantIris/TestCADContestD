// Greedy post-LP local search
#pragma once

#include <chrono>
#include <cstddef>

#include "lp_buffer_dp.hpp"
#include "lp_score.hpp"
#include "lp_types.hpp"

int greedy_post_lp(const char *result_dir, const char *testcase_dir, const LpProblem *pb,
                   const PdDesign *d, const LpBufferChainDp *dp_ss,
                   const LpBufferChainDp *dp_ff, LpSolution *lp_init,
                   const LpMetrics *lp_init_metrics, double time_limit_sec,
                   std::chrono::steady_clock::time_point wall_deadline, char *err,
                   std::size_t err_sz);
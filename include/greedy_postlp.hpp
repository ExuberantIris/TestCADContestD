// Greedy post-LP local search (hold-preserving)
#pragma once

#include <cstddef>
#include "lp_types.hpp"
#include "lp_buffer_dp.hpp"
#include "sa_eval.hpp"
#include "lp_score.hpp"

// 注意：這裡的 LpSolution *lp_init 已經拿掉 const 了！
int greedy_post_lp(const char *result_dir, const char *testcase_dir, const LpProblem *pb,
                   const PdDesign *d, const LpBufferChainDp *dp_ss,
                   const LpBufferChainDp *dp_ff, LpSolution *lp_init,
                   const LpMetrics *lp_init_metrics, double time_limit_sec, char *err,
                   std::size_t err_sz);
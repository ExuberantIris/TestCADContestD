#pragma once

#include <cstddef>
#include <vector>

#include "lp_buffer_dp.hpp"
#include "lp_types.hpp"

struct BranchDpOpts {
    std::vector<double> ss_delays;
    std::vector<double> ff_delays;
};

struct LpEvalCtx {
    std::vector<int> launch_ff;
    std::vector<int> capture_ff;
    std::vector<int> ff_path_br;
    std::vector<int> ff_path_off;
    std::vector<int> node_to_ff;
    std::vector<double> T_ss;
    std::vector<double> T_ff;
    std::vector<double> slack_ss;
    std::vector<double> slack_ff;
    double wns_ss = 0.0;
    double wns_ff = 0.0;
    double tns_ss = 0.0;
    double tns_ff = 0.0;
    double area = 0.0;
    double score = 0.0;
};

bool lp_eval_build_ctx(const LpProblem *pb, const PdDesign *d, LpEvalCtx *ctx);
void lp_eval_build_branch_opts(const LpProblem *pb, const PdDesign *d, std::vector<BranchDpOpts> *opts);
void lp_eval_init_from_design(const LpProblem *pb, const PdDesign *d, const std::vector<BranchDpOpts> &opts,
                              std::vector<double> *d_ss, std::vector<double> *d_ff);
void lp_eval_state(const LpProblem *pb, const PdDesign *d, const std::vector<double> &d_ss,
                   const std::vector<double> &d_ff, const LpBufferChainDp *dp_ss,
                   const LpBufferChainDp *dp_ff, LpEvalCtx *ctx);

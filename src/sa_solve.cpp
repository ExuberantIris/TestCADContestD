#include "sa_solve.hpp"

#include "sa_eval.hpp"
#include "sa_vds.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <random>
#include <vector>

namespace {

using Clock = std::chrono::steady_clock;

double elapsed_sec(const Clock::time_point &t0)
{
    return std::chrono::duration<double>(Clock::now() - t0).count();
}

int pick_weighted_branch(const std::vector<int> &mutable_branches,
                         const std::vector<double> &weights, std::mt19937 &rng)
{
    double sum = 0.0;
    for (int b : mutable_branches)
        sum += weights[static_cast<std::size_t>(b)];
    if (sum < 1e-9)
        return mutable_branches[static_cast<std::size_t>(0)];
    std::uniform_real_distribution<double> dist(0.0, sum);
    double r = dist(rng);
    for (int b : mutable_branches) {
        r -= weights[static_cast<std::size_t>(b)];
        if (r <= 0.0)
            return b;
    }
    return mutable_branches.back();
}

} // namespace

void sa_solution_free(SaSolveResult *out)
{
    if (!out)
        return;
    out->solution.clear();
}

int sa_solve(LpProblem *pb, const PdDesign *d, const LpBufferChainDp *dp_ss,
             const LpBufferChainDp *dp_ff, SaSolveResult *out, char *err, std::size_t err_sz)
{
    if (!pb || !d || !dp_ss || !dp_ff || !out) {
        if (err && err_sz > 0)
            std::snprintf(err, err_sz, "null argument");
        return -1;
    }

    out->solution.clear();
    out->elapsed_sec = 0.0;
    out->timed_out = 0;
    out->use_second_best = 0;
    out->iterations = 0;

    SaPgCtx ctx;
    if (!sa_build_ctx(pb, d, &ctx)) {
        if (err && err_sz > 0)
            std::snprintf(err, err_sz, "failed to build SA context");
        return -1;
    }

    std::vector<BranchDpOpts> opts;
    sa_build_branch_opts(pb, d, &opts);

    const int n_br = static_cast<int>(pb->branches.size());
    std::vector<int> mutable_branches;
    for (int b = 0; b < n_br; b++) {
        if (pb->branches[static_cast<std::size_t>(b)].kind == LpBranchKind::ExistingBuf ||
            pb->branches[static_cast<std::size_t>(b)].kind == LpBranchKind::Insertable)
            mutable_branches.push_back(b);
    }
    if (mutable_branches.empty()) {
        if (err && err_sz > 0)
            std::snprintf(err, err_sz, "no mutable branches");
        return -1;
    }

    std::vector<double> cur_ss(static_cast<std::size_t>(n_br), 0.0);
    std::vector<double> cur_ff(static_cast<std::size_t>(n_br), 0.0);
    sa_init_from_design(pb, d, opts, &cur_ss, &cur_ff);
    sa_eval_state(pb, d, cur_ss, cur_ff, dp_ss, dp_ff, &ctx);

    const Clock::time_point t0 = Clock::now();
    const double time_limit = pb->time_limit_sec;
    const double vds_budget = std::min(90.0, time_limit * 0.15);

    sa_vds_warmstart(pb, d, dp_ss, dp_ff, &cur_ss, &cur_ff, &ctx, opts, vds_budget, err, err_sz);

    double best_score = ctx.score;
    std::vector<double> best_ss = cur_ss;
    std::vector<double> best_ff = cur_ff;

    std::vector<double> trial_ss = cur_ss;
    std::vector<double> trial_ff = cur_ff;
    std::vector<double> branch_w(static_cast<std::size_t>(n_br), 1.0);

    std::mt19937 rng(static_cast<unsigned>(
        std::chrono::steady_clock::now().time_since_epoch().count()));
    std::uniform_real_distribution<double> uni01(0.0, 1.0);

    double temperature = 1.0;
    while (elapsed_sec(t0) < time_limit) {
        out->iterations++;

        sa_branch_weights(pb, ctx, &branch_w);

        trial_ss = cur_ss;
        trial_ff = cur_ff;

        const int b = pick_weighted_branch(mutable_branches, branch_w, rng);
        const auto &o = opts[static_cast<std::size_t>(b)];

        if (uni01(rng) < 0.5 && o.ss_delays.size() > 1) {
            const int idx = std::uniform_int_distribution<int>(
                0, static_cast<int>(o.ss_delays.size()) - 1)(rng);
            trial_ss[static_cast<std::size_t>(b)] = o.ss_delays[static_cast<std::size_t>(idx)];
        } else if (o.ff_delays.size() > 1) {
            const int idx = std::uniform_int_distribution<int>(
                0, static_cast<int>(o.ff_delays.size()) - 1)(rng);
            trial_ff[static_cast<std::size_t>(b)] = o.ff_delays[static_cast<std::size_t>(idx)];
        } else if (o.ss_delays.size() > 1) {
            const int idx = std::uniform_int_distribution<int>(
                0, static_cast<int>(o.ss_delays.size()) - 1)(rng);
            trial_ss[static_cast<std::size_t>(b)] = o.ss_delays[static_cast<std::size_t>(idx)];
        }

        const double old_score = ctx.score;
        SaPgCtx trial_ctx;
        trial_ctx.launch_ff = ctx.launch_ff;
        trial_ctx.capture_ff = ctx.capture_ff;
        trial_ctx.ff_path_br = ctx.ff_path_br;
        trial_ctx.ff_path_off = ctx.ff_path_off;
        trial_ctx.node_to_ff = ctx.node_to_ff;
        sa_eval_state(pb, d, trial_ss, trial_ff, dp_ss, dp_ff, &trial_ctx);

        const double delta = trial_ctx.score - old_score;
        const bool accept =
            delta > 0.0 ||
            (temperature > 1e-9 && uni01(rng) < std::exp(delta / temperature));

        if (accept) {
            cur_ss = trial_ss;
            cur_ff = trial_ff;
            ctx = trial_ctx;
            if (ctx.score > best_score + 1e-12) {
                best_score = ctx.score;
                best_ss = cur_ss;
                best_ff = cur_ff;
            }
        }

        temperature *= 0.9995;
        if (temperature < 1e-4)
            temperature = 1.0;
    }

    out->elapsed_sec = elapsed_sec(t0);
    out->timed_out = (out->elapsed_sec >= time_limit - 1e-6) ? 1 : 0;
    out->use_second_best = 0;
    out->solution.d_ss = best_ss;
    out->solution.d_ff = best_ff;
    out->solution.status = out->timed_out ? 2 : 0;
    out->solution.solver_name = out->timed_out ? "vds_weighted_sa(timed_out)" : "vds_weighted_sa";

    return 0;
}

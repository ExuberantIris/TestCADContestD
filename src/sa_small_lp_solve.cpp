#include "sa_small_lp_solve.hpp"

#include "lp_small_solve.hpp"
#include "sa_config.hpp"
#include "sa_eval.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

namespace {

using Clock = std::chrono::steady_clock;

double elapsed_sec(const Clock::time_point &t0)
{
    return std::chrono::duration<double>(Clock::now() - t0).count();
}

void collect_active_branches(const SaPgCtx &ctx, const std::vector<int> &paths,
                             std::vector<int> *branches)
{
    branches->clear();
    std::vector<char> seen;
    std::vector<int> tmp;
    for (int p : paths) {
        sa_path_branches_for_path(ctx, p, &tmp);
        for (int b : tmp) {
            if (b < 0)
                continue;
            if (static_cast<std::size_t>(b) >= seen.size()) {
                const std::size_t need = static_cast<std::size_t>(b) + 1;
                if (seen.size() < need)
                    seen.resize(need, 0);
            }
            if (!seen[static_cast<std::size_t>(b)]) {
                seen[static_cast<std::size_t>(b)] = 1;
                branches->push_back(b);
            }
        }
    }
    std::sort(branches->begin(), branches->end());
}

} // namespace

int sa_small_lp_solve(LpProblem *pb, const PdDesign *d, const LpBufferChainDp *dp_ss,
                      const LpBufferChainDp *dp_ff, const LpSolution *initial, double sa_time_sec,
                      SaSolveResult *out, char *err, std::size_t err_sz)
{
    if (!pb || !d || !dp_ss || !dp_ff || !initial || !out) {
        if (err && err_sz > 0)
            std::snprintf(err, err_sz, "null argument");
        return -1;
    }

    out->solution.clear();
    out->elapsed_sec = 0.0;
    out->timed_out = 0;
    out->use_second_best = 0;
    out->iterations = 0;
    out->lp_init_ok = 0;
    out->lp_init_sec = 0.0;

    SaPgCtx ctx;
    if (!sa_build_ctx(pb, d, &ctx)) {
        if (err && err_sz > 0)
            std::snprintf(err, err_sz, "failed to build SA context");
        return -1;
    }

    std::vector<BranchDpOpts> opts;
    sa_build_branch_opts(pb, d, &opts);

    const int n_paths = static_cast<int>(pb->path_ids.size());
    std::vector<double> cur_ss = initial->d_ss;
    std::vector<double> cur_ff = initial->d_ff;
    if (cur_ss.size() != pb->branches.size())
        sa_init_from_design(pb, d, opts, &cur_ss, &cur_ff);

    sa_eval_state(pb, d, cur_ss, cur_ff, dp_ss, dp_ff, &ctx);

    double best_score = ctx.score;
    std::vector<double> best_ss = cur_ss;
    std::vector<double> best_ff = cur_ff;

    const Clock::time_point sa_t0 = Clock::now();
    int no_improve_cycles = 0;
    int stalled = 0;

    std::vector<int> violating;
    std::vector<int> selected_paths;
    std::vector<int> active_branches;
    std::vector<double> trial_ss;
    std::vector<double> trial_ff;

    while (elapsed_sec(sa_t0) < sa_time_sec) {
        out->iterations++;

        sa_eval_state(pb, d, cur_ss, cur_ff, dp_ss, dp_ff, &ctx);

        violating.clear();
        violating.reserve(static_cast<std::size_t>(n_paths));
        for (int p = 0; p < n_paths; p++) {
            if (ctx.slack_ss[static_cast<std::size_t>(p)] < 0.0 ||
                ctx.slack_ff[static_cast<std::size_t>(p)] < 0.0)
                violating.push_back(p);
        }

        if (violating.empty())
            break;

        std::sort(violating.begin(), violating.end(), [&ctx](int a, int b) {
            const double wa = sa_path_violation_weight(ctx, a);
            const double wb = sa_path_violation_weight(ctx, b);
            if (wa != wb)
                return wa > wb;
            return a < b;
        });

        const int pick_k = std::min(SaConfig::kSmallLpPathCount, static_cast<int>(violating.size()));
        selected_paths.assign(violating.begin(), violating.begin() + pick_k);
        collect_active_branches(ctx, selected_paths, &active_branches);

        if (active_branches.empty()) {
            no_improve_cycles++;
            if (no_improve_cycles >= SaConfig::kSmallLpNoImproveLimit) {
                stalled = 1;
                break;
            }
            continue;
        }

        if (lp_solve_subset(pb, d, ctx, dp_ss, dp_ff, cur_ss, cur_ff, active_branches,
                            selected_paths, SaConfig::kSmallLpTimeLimitSec, &trial_ss, &trial_ff,
                            err, err_sz) != 0)
            return -1;

        cur_ss = std::move(trial_ss);
        cur_ff = std::move(trial_ff);
        sa_eval_state(pb, d, cur_ss, cur_ff, dp_ss, dp_ff, &ctx);

        if (ctx.score > best_score + 1e-12) {
            best_score = ctx.score;
            best_ss = cur_ss;
            best_ff = cur_ff;
            no_improve_cycles = 0;
        } else {
            no_improve_cycles++;
            if (no_improve_cycles >= SaConfig::kSmallLpNoImproveLimit) {
                stalled = 1;
                break;
            }
        }
    }

    out->elapsed_sec = elapsed_sec(sa_t0);
    out->timed_out = (out->elapsed_sec >= sa_time_sec - 1e-6) ? 1 : 0;
    out->solution.d_ss = best_ss;
    out->solution.d_ff = best_ff;
    out->solution.status = out->timed_out ? 2 : 0;
    if (stalled)
        out->solution.solver_name = "small_lp_sa(stall)";
    else if (violating.empty())
        out->solution.solver_name = "small_lp_sa(converged)";
    else
        out->solution.solver_name =
            out->timed_out ? "small_lp_sa(timed_out)" : "small_lp_sa";

    return 0;
}

void sa_solution_free(SaSolveResult *out)
{
    if (!out)
        return;
    out->solution.clear();
}

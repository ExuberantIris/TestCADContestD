#include "sa_vds.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <vector>

namespace {

using Clock = std::chrono::steady_clock;

double elapsed_sec(const Clock::time_point &t0)
{
    return std::chrono::duration<double>(Clock::now() - t0).count();
}

/** Next delay in opts strictly greater than cur (or max). */
double next_higher(double cur, const std::vector<double> &lst)
{
    double best = cur;
    bool found = false;
    for (double x : lst) {
        if (x > cur + 1e-9 && (!found || x < best)) {
            best = x;
            found = true;
        }
    }
    if (!found) {
        for (double x : lst)
            best = std::max(best, x);
    }
    return best;
}

/** Next delay in opts strictly less than cur (or min). */
double next_lower(double cur, const std::vector<double> &lst)
{
    double best = cur;
    bool found = false;
    for (double x : lst) {
        if (x < cur - 1e-9 && (!found || x > best)) {
            best = x;
            found = true;
        }
    }
    if (!found) {
        for (double x : lst)
            best = std::min(best, x);
    }
    return best;
}

void path_branches(const SaPgCtx &ctx, int ff_idx, std::vector<int> *out)
{
    out->clear();
    if (ff_idx < 0)
        return;
    for (int k = ctx.ff_path_off[static_cast<std::size_t>(ff_idx)];
         k < ctx.ff_path_off[static_cast<std::size_t>(ff_idx + 1)]; k++)
        out->push_back(ctx.ff_path_br[static_cast<std::size_t>(k)]);
}

} // namespace

int sa_vds_warmstart(LpProblem *pb, const PdDesign *d, const LpBufferChainDp *dp_ss,
                    const LpBufferChainDp *dp_ff, std::vector<double> *cur_ss,
                    std::vector<double> *cur_ff, SaPgCtx *ctx,
                    const std::vector<BranchDpOpts> &opts, double time_budget_sec, char * /*err*/,
                    std::size_t /*err_sz*/)
{
    const Clock::time_point t0 = Clock::now();
    const int n_paths = static_cast<int>(pb->path_ids.size());
    const int n_br = static_cast<int>(pb->branches.size());
    int stall = 0;

    while (elapsed_sec(t0) < time_budget_sec && stall < 8) {
        sa_eval_state(pb, d, *cur_ss, *cur_ff, dp_ss, dp_ff, ctx);
        if (ctx->wns_ss >= -1e-6 && ctx->wns_ff >= -1e-6) {
            stall = 8;
            break;
        }

        int worst_p = -1;
        double worst_slack = 0.0;
        const bool fix_setup = ctx->wns_ss < ctx->wns_ff - 1e-9 || ctx->wns_ss < 0.0;

        for (int p = 0; p < n_paths; p++) {
            const double s =
                fix_setup ? ctx->slack_ss[static_cast<std::size_t>(p)]
                          : ctx->slack_ff[static_cast<std::size_t>(p)];
            if (s >= 0.0)
                continue;
            if (worst_p < 0 || s < worst_slack) {
                worst_p = p;
                worst_slack = s;
            }
        }
        if (worst_p < 0) {
            stall++;
            continue;
        }

        std::vector<int> branches;
        const int li = ctx->launch_ff[static_cast<std::size_t>(worst_p)];
        const int ci = ctx->capture_ff[static_cast<std::size_t>(worst_p)];
        if (fix_setup)
            path_branches(*ctx, ci, &branches);
        else
            path_branches(*ctx, li, &branches);

        if (branches.empty()) {
            stall++;
            continue;
        }

        const double old_score = ctx->score;
        bool improved = false;

        for (int b : branches) {
            if (b < 0 || b >= n_br)
                continue;
            const auto &o = opts[static_cast<std::size_t>(b)];
            std::vector<double> trial_ss = *cur_ss;
            std::vector<double> trial_ff = *cur_ff;

            if (fix_setup) {
                trial_ss[static_cast<std::size_t>(b)] =
                    next_higher((*cur_ss)[static_cast<std::size_t>(b)], o.ss_delays);
            } else {
                trial_ff[static_cast<std::size_t>(b)] =
                    next_higher((*cur_ff)[static_cast<std::size_t>(b)], o.ff_delays);
            }

            SaPgCtx trial = *ctx;
            sa_eval_state(pb, d, trial_ss, trial_ff, dp_ss, dp_ff, &trial);
            if (trial.score > old_score + 1e-9) {
                *cur_ss = std::move(trial_ss);
                *cur_ff = std::move(trial_ff);
                *ctx = trial;
                improved = true;
                break;
            }
        }

        if (!improved) {
            for (int b : branches) {
                if (b < 0 || b >= n_br)
                    continue;
                const auto &o = opts[static_cast<std::size_t>(b)];
                std::vector<double> trial_ss = *cur_ss;
                std::vector<double> trial_ff = *cur_ff;
                if (fix_setup)
                    trial_ff[static_cast<std::size_t>(b)] =
                        next_lower((*cur_ff)[static_cast<std::size_t>(b)], o.ff_delays);
                else
                    trial_ss[static_cast<std::size_t>(b)] =
                        next_lower((*cur_ss)[static_cast<std::size_t>(b)], o.ss_delays);

                SaPgCtx trial = *ctx;
                sa_eval_state(pb, d, trial_ss, trial_ff, dp_ss, dp_ff, &trial);
                if (trial.score > old_score + 1e-9) {
                    *cur_ss = std::move(trial_ss);
                    *cur_ff = std::move(trial_ff);
                    *ctx = trial;
                    improved = true;
                    break;
                }
            }
        }

        if (improved)
            stall = 0;
        else
            stall++;
    }

    sa_eval_state(pb, d, *cur_ss, *cur_ff, dp_ss, dp_ff, ctx);
    return 0;
}

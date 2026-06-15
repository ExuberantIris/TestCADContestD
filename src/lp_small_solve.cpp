#include "lp_small_solve.hpp"

#include "lp_score.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <vector>

#include "lp_buffer_dp.hpp"

namespace {

using Clock = std::chrono::steady_clock;

double elapsed_sec(const Clock::time_point &t0)
{
    return std::chrono::duration<double>(Clock::now() - t0).count();
}

void eval_subset_score(const LpProblem *pb, const PdDesign *d, const SaPgCtx &ctx,
                       const LpBufferChainDp *dp_ss, const LpBufferChainDp *dp_ff,
                       const std::vector<double> &d_ss, const std::vector<double> &d_ff,
                       double *score_out)
{
    SaPgCtx trial = ctx;
    sa_eval_state(pb, d, d_ss, d_ff, dp_ss, dp_ff, &trial);
    *score_out = trial.score;
}

void project_active_bounds(const LpProblem *pb, const std::vector<int> &active_branches,
                           std::vector<double> *d_ss, std::vector<double> *d_ff)
{
    for (int b : active_branches) {
        const LpBranch &br = pb->branches[static_cast<std::size_t>(b)];
        auto &ss = (*d_ss)[static_cast<std::size_t>(b)];
        auto &ff = (*d_ff)[static_cast<std::size_t>(b)];
        ss = std::clamp(ss, br.d_ss_min, br.d_ss_max);
        ff = std::clamp(ff, br.d_ff_min, br.d_ff_max);
    }
}

} // namespace

int lp_solve_subset(const LpProblem *pb, const PdDesign *d, const SaPgCtx &ctx,
                    const LpBufferChainDp *dp_ss, const LpBufferChainDp *dp_ff,
                    const std::vector<double> &fixed_ss, const std::vector<double> &fixed_ff,
                    const std::vector<int> &active_branches, const std::vector<int> &active_paths,
                    double time_limit_sec, std::vector<double> *out_ss, std::vector<double> *out_ff,
                    char *err, std::size_t err_sz)
{
    if (!pb || !d || !out_ss || !out_ff) {
        if (err && err_sz > 0)
            std::snprintf(err, err_sz, "null argument");
        return -1;
    }
    if (active_branches.empty() || active_paths.empty()) {
        *out_ss = fixed_ss;
        *out_ff = fixed_ff;
        return 0;
    }

    const int n_br = static_cast<int>(pb->branches.size());
    const double eps = 1e-6;
    const double step = 0.004;

    std::vector<char> active(static_cast<std::size_t>(n_br), 0);
    for (int b : active_branches)
        active[static_cast<std::size_t>(b)] = 1;

    std::vector<double> d_ss = fixed_ss;
    std::vector<double> d_ff = fixed_ff;
    std::vector<double> g_ss(static_cast<std::size_t>(n_br), 0.0);
    std::vector<double> g_ff(static_cast<std::size_t>(n_br), 0.0);
    std::vector<double> best_ss = d_ss;
    std::vector<double> best_ff = d_ff;

    double best_score = -1e30;
    eval_subset_score(pb, d, ctx, dp_ss, dp_ff, d_ss, d_ff, &best_score);

    SaPgCtx work = ctx;
    const Clock::time_point t0 = Clock::now();

    while (elapsed_sec(t0) < time_limit_sec) {
        sa_eval_state(pb, d, d_ss, d_ff, dp_ss, dp_ff, &work);

        std::fill(g_ss.begin(), g_ss.end(), 0.0);
        std::fill(g_ff.begin(), g_ff.end(), 0.0);

        for (int p : active_paths) {
            const double ss_sl = work.slack_ss[static_cast<std::size_t>(p)];
            const double ff_sl = work.slack_ff[static_cast<std::size_t>(p)];

            double gw_ss = 0.0, gt_ss = 0.0, gw_ff = 0.0, gt_ff = 0.0;

            if (ss_sl < 0.0) {
                const double v = -ss_sl;
                gt_ss = -v / (std::fabs(pb->tns_ss_ori) + eps);
                if (ss_sl <= work.wns_ss + 1e-9)
                    gw_ss = -v / (std::fabs(pb->wns_ss_ori) + eps);
            }
            if (ff_sl < 0.0) {
                const double v = -ff_sl;
                gt_ff = -v / (std::fabs(pb->tns_ff_ori) + eps);
                if (ff_sl <= work.wns_ff + 1e-9)
                    gw_ff = -v / (std::fabs(pb->wns_ff_ori) + eps);
            }

            const int li = ctx.launch_ff[static_cast<std::size_t>(p)];
            const int ci = ctx.capture_ff[static_cast<std::size_t>(p)];

            if (ci >= 0) {
                for (int k = ctx.ff_path_off[static_cast<std::size_t>(ci)];
                     k < ctx.ff_path_off[static_cast<std::size_t>(ci + 1)]; k++) {
                    const int b = ctx.ff_path_br[static_cast<std::size_t>(k)];
                    if (!active[static_cast<std::size_t>(b)])
                        continue;
                    g_ss[static_cast<std::size_t>(b)] += gw_ss + gt_ss;
                    g_ff[static_cast<std::size_t>(b)] += gw_ff * 0.5 + gt_ff * 0.5;
                }
            }
            if (li >= 0) {
                for (int k = ctx.ff_path_off[static_cast<std::size_t>(li)];
                     k < ctx.ff_path_off[static_cast<std::size_t>(li + 1)]; k++) {
                    const int b = ctx.ff_path_br[static_cast<std::size_t>(k)];
                    if (!active[static_cast<std::size_t>(b)])
                        continue;
                    g_ss[static_cast<std::size_t>(b)] -= gw_ss + gt_ss;
                    g_ff[static_cast<std::size_t>(b)] -= gw_ff + gt_ff;
                }
            }
        }

        for (int b : active_branches) {
            d_ss[static_cast<std::size_t>(b)] -= step * g_ss[static_cast<std::size_t>(b)];
            d_ff[static_cast<std::size_t>(b)] -= step * g_ff[static_cast<std::size_t>(b)];
        }
        project_active_bounds(pb, active_branches, &d_ss, &d_ff);

        double score = 0.0;
        eval_subset_score(pb, d, ctx, dp_ss, dp_ff, d_ss, d_ff, &score);
        if (score > best_score + 1e-12) {
            best_score = score;
            best_ss = d_ss;
            best_ff = d_ff;
        }
    }

    *out_ss = std::move(best_ss);
    *out_ff = std::move(best_ff);
    return 0;
}

#include "sa_path_solve.hpp"

#include "sa_config.hpp"
#include "sa_eval.hpp"

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
    if (!found)
        for (double x : lst)
            best = std::max(best, x);
    return best;
}

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
    if (!found)
        for (double x : lst)
            best = std::min(best, x);
    return best;
}

bool apply_path_move(int path_idx, const SaPgCtx &ctx, const std::vector<BranchDpOpts> &opts,
                     const SaPathIndex &path_index, const std::vector<int> &branch_stall,
                     int branch_stall_limit, bool setup_only, std::vector<double> *d_ss,
                     std::vector<double> *d_ff, std::mt19937 &rng, int *out_branch)
{
    const bool fix_setup = ctx.slack_ss[static_cast<std::size_t>(path_idx)] < 0.0;
    const bool fix_hold = ctx.slack_ff[static_cast<std::size_t>(path_idx)] < 0.0;
    if (setup_only) {
        if (!fix_setup)
            return false;
    } else if (!fix_setup && !fix_hold) {
        return false;
    }

    std::vector<int> branches;
    if (fix_setup || setup_only)
        sa_path_branches_capture(ctx, path_idx, &branches);
    else
        sa_path_branches_launch(ctx, path_idx, &branches);

    if (branches.empty())
        return false;

    std::vector<int> eligible;
    eligible.reserve(branches.size());
    for (int b : branches) {
        if (branch_stall[static_cast<std::size_t>(b)] < branch_stall_limit)
            eligible.push_back(b);
    }
    if (eligible.empty())
        return false;

    int min_impact = static_cast<int>(path_index.branch_path_count.size()) + 1;
    for (int b : eligible) {
        const int cnt = path_index.branch_path_count[static_cast<std::size_t>(b)];
        min_impact = std::min(min_impact, cnt);
    }

    std::vector<int> min_branches;
    min_branches.reserve(eligible.size());
    for (int b : eligible) {
        if (path_index.branch_path_count[static_cast<std::size_t>(b)] == min_impact)
            min_branches.push_back(b);
    }

    const int b = min_branches[static_cast<std::size_t>(
        std::uniform_int_distribution<int>(0, static_cast<int>(min_branches.size()) - 1)(rng))];
    const auto &o = opts[static_cast<std::size_t>(b)];

    if ((fix_setup || setup_only) && o.ss_delays.size() > 1) {
        if (std::uniform_int_distribution<int>(0, 1)(rng) == 0)
            (*d_ss)[static_cast<std::size_t>(b)] =
                next_higher((*d_ss)[static_cast<std::size_t>(b)], o.ss_delays);
        else
            (*d_ff)[static_cast<std::size_t>(b)] =
                next_lower((*d_ff)[static_cast<std::size_t>(b)], o.ff_delays);
    } else if (!setup_only && fix_hold && o.ff_delays.size() > 1) {
        if (std::uniform_int_distribution<int>(0, 1)(rng) == 0)
            (*d_ff)[static_cast<std::size_t>(b)] =
                next_higher((*d_ff)[static_cast<std::size_t>(b)], o.ff_delays);
        else
            (*d_ss)[static_cast<std::size_t>(b)] =
                next_lower((*d_ss)[static_cast<std::size_t>(b)], o.ss_delays);
    } else if (!setup_only && o.ss_delays.size() > 1) {
        (*d_ss)[static_cast<std::size_t>(b)] =
            next_higher((*d_ss)[static_cast<std::size_t>(b)], o.ss_delays);
    } else if (!setup_only && o.ff_delays.size() > 1) {
        (*d_ff)[static_cast<std::size_t>(b)] =
            next_higher((*d_ff)[static_cast<std::size_t>(b)], o.ff_delays);
    } else {
        return false;
    }

    if (out_branch)
        *out_branch = b;
    return true;
}

} // namespace

int sa_path_solve(LpProblem *pb, const PdDesign *d, const LpBufferChainDp *dp_ss,
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

    SaPgCtx ctx;
    if (!sa_build_ctx(pb, d, &ctx)) {
        if (err && err_sz > 0)
            std::snprintf(err, err_sz, "failed to build SA context");
        return -1;
    }

    SaPathIndex path_index;
    if (!sa_build_path_index(pb, ctx, &path_index)) {
        if (err && err_sz > 0)
            std::snprintf(err, err_sz, "failed to build path index");
        return -1;
    }

    std::vector<BranchDpOpts> opts;
    sa_build_branch_opts(pb, d, &opts);

    const int n_br = static_cast<int>(pb->branches.size());
    const int n_paths = static_cast<int>(pb->path_ids.size());

    std::vector<double> cur_ss = initial->d_ss;
    std::vector<double> cur_ff = initial->d_ff;
    if (static_cast<int>(cur_ss.size()) != n_br)
        sa_init_from_design(pb, d, opts, &cur_ss, &cur_ff);

    sa_eval_state(pb, d, cur_ss, cur_ff, dp_ss, dp_ff, &ctx);

    double best_score = ctx.score;
    double best_tns_ss = ctx.tns_ss;
    std::vector<double> best_ss = cur_ss;
    std::vector<double> best_ff = cur_ff;

    std::mt19937 rng(static_cast<unsigned>(
        std::chrono::steady_clock::now().time_since_epoch().count()));
    std::uniform_real_distribution<double> uni01(0.0, 1.0);

    const Clock::time_point sa_t0 = Clock::now();
    double temperature = 1.0;
    int no_improve_iters = 0;
    int stalled = 0;
    int top_pool_size = SaConfig::kTopPathPoolInit;
    std::vector<int> branch_stall(static_cast<std::size_t>(n_br), 0);

    while (elapsed_sec(sa_t0) < sa_time_sec) {
        out->iterations++;

        const bool setup_only = out->iterations <= SaConfig::kSetupTnsOnlyIters;

        if (out->iterations == SaConfig::kSetupTnsOnlyIters + 1) {
            no_improve_iters = 0;
            best_score = ctx.score;
        }

        if (SaConfig::kTopPathPoolGrowEvery > 0 && out->iterations > 1 &&
            (out->iterations - 1) % SaConfig::kTopPathPoolGrowEvery == 0)
            top_pool_size += SaConfig::kTopPathPoolGrowStep;

        sa_eval_state(pb, d, cur_ss, cur_ff, dp_ss, dp_ff, &ctx);

        std::vector<int> violating;
        violating.reserve(static_cast<std::size_t>(n_paths));
        for (int p = 0; p < n_paths; p++) {
            if (setup_only) {
                if (ctx.slack_ss[static_cast<std::size_t>(p)] < 0.0)
                    violating.push_back(p);
            } else if (ctx.slack_ss[static_cast<std::size_t>(p)] < 0.0 ||
                       ctx.slack_ff[static_cast<std::size_t>(p)] < 0.0) {
                violating.push_back(p);
            }
        }

        if (violating.empty()) {
            if (setup_only)
                continue;
            break;
        }

        std::sort(violating.begin(), violating.end(), [&ctx, setup_only](int a, int b) {
            const double wa = setup_only ? sa_path_setup_violation_weight(ctx, a)
                                         : sa_path_violation_weight(ctx, a);
            const double wb = setup_only ? sa_path_setup_violation_weight(ctx, b)
                                         : sa_path_violation_weight(ctx, b);
            if (wa != wb)
                return wa > wb;
            return a < b;
        });

        const int pool_n = std::min(top_pool_size, static_cast<int>(violating.size()));
        std::vector<int> pool(violating.begin(), violating.begin() + pool_n);

        const int path_idx = pool[static_cast<std::size_t>(
            std::uniform_int_distribution<int>(0, pool_n - 1)(rng))];

        std::vector<double> trial_ss = cur_ss;
        std::vector<double> trial_ff = cur_ff;
        int moved_branch = -1;
        if (!apply_path_move(path_idx, ctx, opts, path_index, branch_stall,
                             SaConfig::kBranchNoImproveLimit, setup_only, &trial_ss, &trial_ff, rng,
                             &moved_branch))
            continue;

        const double old_obj = setup_only ? ctx.tns_ss : ctx.score;
        SaPgCtx trial_ctx;
        trial_ctx.launch_ff = ctx.launch_ff;
        trial_ctx.capture_ff = ctx.capture_ff;
        trial_ctx.ff_path_br = ctx.ff_path_br;
        trial_ctx.ff_path_off = ctx.ff_path_off;
        trial_ctx.node_to_ff = ctx.node_to_ff;
        sa_eval_state(pb, d, trial_ss, trial_ff, dp_ss, dp_ff, &trial_ctx);

        const double new_obj = setup_only ? trial_ctx.tns_ss : trial_ctx.score;
        const double delta = new_obj - old_obj;
        const bool accept =
            delta > 0.0 ||
            (temperature > 1e-9 && uni01(rng) < std::exp(delta / temperature));

        bool improved_best = false;
        if (accept) {
            cur_ss = std::move(trial_ss);
            cur_ff = std::move(trial_ff);
            ctx = trial_ctx;
            if (setup_only) {
                if (ctx.tns_ss > best_tns_ss + 1e-12) {
                    best_tns_ss = ctx.tns_ss;
                    best_score = ctx.score;
                    best_ss = cur_ss;
                    best_ff = cur_ff;
                    improved_best = true;
                }
            } else if (ctx.score > best_score + 1e-12) {
                best_score = ctx.score;
                best_ss = cur_ss;
                best_ff = cur_ff;
                improved_best = true;
            }
        }

        if (moved_branch >= 0) {
            if (improved_best)
                branch_stall[static_cast<std::size_t>(moved_branch)] = 0;
            else
                branch_stall[static_cast<std::size_t>(moved_branch)]++;
        }

        if (improved_best) {
            no_improve_iters = 0;
        } else {
            no_improve_iters++;
            if (no_improve_iters >= SaConfig::kNoImproveLimit) {
                stalled = 1;
                break;
            }
        }

        temperature *= 0.9995;
        if (temperature < 1e-4)
            temperature = 1.0;
    }

    out->elapsed_sec = elapsed_sec(sa_t0);
    out->timed_out = (out->elapsed_sec >= sa_time_sec - 1e-6) ? 1 : 0;
    out->solution.d_ss = best_ss;
    out->solution.d_ff = best_ff;
    out->solution.status = out->timed_out ? 2 : 0;
    if (stalled)
        out->solution.solver_name = "path_heap_sa(stall)";
    else
        out->solution.solver_name =
            out->timed_out ? "path_heap_sa(timed_out)" : "path_heap_sa";

    return 0;
}

void sa_solution_free(SaSolveResult *out)
{
    if (!out)
        return;
    out->solution.clear();
}

#include "sa_path_solve.hpp"

#include "sa_eval.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <queue>
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
                     std::vector<double> *d_ss, std::vector<double> *d_ff, std::mt19937 &rng)
{
    const bool fix_setup = ctx.slack_ss[static_cast<std::size_t>(path_idx)] < 0.0;
    const bool fix_hold = ctx.slack_ff[static_cast<std::size_t>(path_idx)] < 0.0;
    if (!fix_setup && !fix_hold)
        return false;

    std::vector<int> branches;
    if (fix_setup)
        sa_path_branches_capture(ctx, path_idx, &branches);
    else
        sa_path_branches_launch(ctx, path_idx, &branches);

    if (branches.empty())
        return false;

    const int b = branches[static_cast<std::size_t>(
        std::uniform_int_distribution<int>(0, static_cast<int>(branches.size()) - 1)(rng))];
    const auto &o = opts[static_cast<std::size_t>(b)];

    if (fix_setup && o.ss_delays.size() > 1) {
        if (std::uniform_int_distribution<int>(0, 1)(rng) == 0)
            (*d_ss)[static_cast<std::size_t>(b)] =
                next_higher((*d_ss)[static_cast<std::size_t>(b)], o.ss_delays);
        else
            (*d_ff)[static_cast<std::size_t>(b)] =
                next_lower((*d_ff)[static_cast<std::size_t>(b)], o.ff_delays);
    } else if (fix_hold && o.ff_delays.size() > 1) {
        if (std::uniform_int_distribution<int>(0, 1)(rng) == 0)
            (*d_ff)[static_cast<std::size_t>(b)] =
                next_higher((*d_ff)[static_cast<std::size_t>(b)], o.ff_delays);
        else
            (*d_ss)[static_cast<std::size_t>(b)] =
                next_lower((*d_ss)[static_cast<std::size_t>(b)], o.ss_delays);
    } else if (o.ss_delays.size() > 1) {
        (*d_ss)[static_cast<std::size_t>(b)] =
            next_higher((*d_ss)[static_cast<std::size_t>(b)], o.ss_delays);
    } else if (o.ff_delays.size() > 1) {
        (*d_ff)[static_cast<std::size_t>(b)] =
            next_higher((*d_ff)[static_cast<std::size_t>(b)], o.ff_delays);
    } else {
        return false;
    }
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
    std::vector<double> best_ss = cur_ss;
    std::vector<double> best_ff = cur_ff;

    std::mt19937 rng(static_cast<unsigned>(
        std::chrono::steady_clock::now().time_since_epoch().count()));
    std::uniform_real_distribution<double> uni01(0.0, 1.0);

    const Clock::time_point sa_t0 = Clock::now();
    double temperature = 1.0;

    while (elapsed_sec(sa_t0) < sa_time_sec) {
        out->iterations++;

        sa_eval_state(pb, d, cur_ss, cur_ff, dp_ss, dp_ff, &ctx);

        std::vector<int> violating;
        violating.reserve(static_cast<std::size_t>(n_paths));
        for (int p = 0; p < n_paths; p++) {
            if (ctx.slack_ss[static_cast<std::size_t>(p)] < 0.0 ||
                ctx.slack_ff[static_cast<std::size_t>(p)] < 0.0)
                violating.push_back(p);
        }

        if (violating.empty())
            break;

        using HeapItem = std::pair<double, int>;
        auto cmp = [](const HeapItem &a, const HeapItem &b) { return a.first < b.first; };
        std::priority_queue<HeapItem, std::vector<HeapItem>, decltype(cmp)> heap(cmp);

        for (int p : violating) {
            const double w = sa_path_violation_weight(ctx, p);
            if (w > 1e-12)
                heap.push({w, p});
        }

        std::vector<int> pool;
        pool.reserve(20);

        for (int i = 0; i < 10 && !heap.empty(); i++) {
            pool.push_back(heap.top().second);
            heap.pop();
        }

        std::uniform_int_distribution<int> pick_v(0, static_cast<int>(violating.size()) - 1);
        for (int i = 0; i < 10 && static_cast<int>(pool.size()) < 20; i++)
            pool.push_back(violating[static_cast<std::size_t>(pick_v(rng))]);

        const int path_idx = pool[static_cast<std::size_t>(
            std::uniform_int_distribution<int>(0, static_cast<int>(pool.size()) - 1)(rng))];

        std::vector<double> trial_ss = cur_ss;
        std::vector<double> trial_ff = cur_ff;
        if (!apply_path_move(path_idx, ctx, opts, &trial_ss, &trial_ff, rng))
            continue;

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
            cur_ss = std::move(trial_ss);
            cur_ff = std::move(trial_ff);
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

    out->elapsed_sec = elapsed_sec(sa_t0);
    out->timed_out = (out->elapsed_sec >= sa_time_sec - 1e-6) ? 1 : 0;
    out->solution.d_ss = best_ss;
    out->solution.d_ff = best_ff;
    out->solution.status = out->timed_out ? 2 : 0;
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

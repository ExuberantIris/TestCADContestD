#include "lp_mo_init.hpp"

#include "lp_score.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <vector>

namespace {

using Clock = std::chrono::steady_clock;

struct LpPgCtx {
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
    double obj = 0.0;
};

double elapsed_sec(const Clock::time_point &t0)
{
    return std::chrono::duration<double>(Clock::now() - t0).count();
}

bool build_pg_ctx(const LpProblem *pb, const PdDesign *d, LpPgCtx *ctx)
{
    const int n_ff = static_cast<int>(pb->ff_node_ids.size());
    const int n_paths = static_cast<int>(pb->path_ids.size());
    const int n_br = static_cast<int>(pb->branches.size());

    ctx->launch_ff.assign(static_cast<std::size_t>(n_paths), -1);
    ctx->capture_ff.assign(static_cast<std::size_t>(n_paths), -1);
    ctx->node_to_ff.assign(static_cast<std::size_t>(d->n_nodes), -1);

    for (int f = 0; f < n_ff; f++)
        ctx->node_to_ff[static_cast<std::size_t>(pb->ff_node_ids[static_cast<std::size_t>(f)])] = f;

    for (int p = 0; p < n_paths; p++) {
        const PdPath *path = &d->paths[pb->path_ids[static_cast<std::size_t>(p)]];
        ctx->launch_ff[static_cast<std::size_t>(p)] =
            (path->launch_id >= 0) ? ctx->node_to_ff[static_cast<std::size_t>(path->launch_id)] : -1;
        ctx->capture_ff[static_cast<std::size_t>(p)] =
            (path->capture_id >= 0) ? ctx->node_to_ff[static_cast<std::size_t>(path->capture_id)] : -1;
    }

    ctx->ff_path_off.assign(static_cast<std::size_t>(n_ff + 1), 0);
    ctx->ff_path_br.clear();
    int off = 0;
    for (int f = 0; f < n_ff; f++) {
        ctx->ff_path_off[static_cast<std::size_t>(f)] = off;
        int cur = pb->ff_node_ids[static_cast<std::size_t>(f)];
        int steps = 0;
        while (cur >= 0 && steps++ < d->n_nodes) {
            const int parent = d->nodes[cur].parent;
            if (parent < 0)
                break;
            for (int b = 0; b < n_br; b++) {
                const LpBranch &br = pb->branches[static_cast<std::size_t>(b)];
                if (br.child_node == cur && br.parent_node == parent) {
                    ctx->ff_path_br.push_back(b);
                    off++;
                    break;
                }
            }
            cur = parent;
        }
    }
    ctx->ff_path_off[static_cast<std::size_t>(n_ff)] = off;

    ctx->T_ss.assign(static_cast<std::size_t>(n_ff), 0.0);
    ctx->T_ff.assign(static_cast<std::size_t>(n_ff), 0.0);
    ctx->slack_ss.assign(static_cast<std::size_t>(n_paths), 0.0);
    ctx->slack_ff.assign(static_cast<std::size_t>(n_paths), 0.0);
    return true;
}

void eval_solution(const LpProblem *pb, const PdDesign *d, const std::vector<double> &d_ss,
                   const std::vector<double> &d_ff, LpPgCtx *ctx)
{
    const int n_ff = static_cast<int>(pb->ff_node_ids.size());
    const int n_paths = static_cast<int>(pb->path_ids.size());
    const int n_br = static_cast<int>(pb->branches.size());

    for (int f = 0; f < n_ff; f++) {
        ctx->T_ss[static_cast<std::size_t>(f)] = 0.0;
        ctx->T_ff[static_cast<std::size_t>(f)] = 0.0;
        for (int k = ctx->ff_path_off[static_cast<std::size_t>(f)];
             k < ctx->ff_path_off[static_cast<std::size_t>(f + 1)]; k++) {
            const int b = ctx->ff_path_br[static_cast<std::size_t>(k)];
            ctx->T_ss[static_cast<std::size_t>(f)] += d_ss[static_cast<std::size_t>(b)];
            ctx->T_ff[static_cast<std::size_t>(f)] += d_ff[static_cast<std::size_t>(b)];
        }
    }

    ctx->wns_ss = 1e30;
    ctx->wns_ff = 1e30;
    ctx->tns_ss = 0.0;
    ctx->tns_ff = 0.0;

    for (int p = 0; p < n_paths; p++) {
        const int li = ctx->launch_ff[static_cast<std::size_t>(p)];
        const int ci = ctx->capture_ff[static_cast<std::size_t>(p)];
        double t_ss_l = 0.0, t_ss_c = 0.0, t_ff_l = 0.0, t_ff_c = 0.0;

        if (li >= 0) {
            t_ss_l = ctx->T_ss[static_cast<std::size_t>(li)];
            t_ff_l = ctx->T_ff[static_cast<std::size_t>(li)];
        }
        if (ci >= 0) {
            t_ss_c = ctx->T_ss[static_cast<std::size_t>(ci)];
            t_ff_c = ctx->T_ff[static_cast<std::size_t>(ci)];
        }

        ctx->slack_ss[static_cast<std::size_t>(p)] =
            d->clock_period - d->t_setup -
            d->paths[pb->path_ids[static_cast<std::size_t>(p)]].data_ss + (t_ss_c - t_ss_l);
        ctx->slack_ff[static_cast<std::size_t>(p)] =
            d->paths[pb->path_ids[static_cast<std::size_t>(p)]].data_ff - d->t_hold -
            (t_ff_c - t_ff_l);

        if (ctx->slack_ss[static_cast<std::size_t>(p)] < ctx->wns_ss)
            ctx->wns_ss = ctx->slack_ss[static_cast<std::size_t>(p)];
        if (ctx->slack_ss[static_cast<std::size_t>(p)] < 0.0)
            ctx->tns_ss += ctx->slack_ss[static_cast<std::size_t>(p)];
        if (ctx->slack_ff[static_cast<std::size_t>(p)] < ctx->wns_ff)
            ctx->wns_ff = ctx->slack_ff[static_cast<std::size_t>(p)];
        if (ctx->slack_ff[static_cast<std::size_t>(p)] < 0.0)
            ctx->tns_ff += ctx->slack_ff[static_cast<std::size_t>(p)];
    }

    if (ctx->wns_ss > 1e29)
        ctx->wns_ss = 0.0;
    if (ctx->wns_ff > 1e29)
        ctx->wns_ff = 0.0;

    ctx->area = 0.0;
    for (int b = 0; b < n_br; b++) {
        const LpBranch &br = pb->branches[static_cast<std::size_t>(b)];
        if (br.kind == LpBranchKind::ExistingBuf)
            ctx->area += br.area_per_ss;
        else if (d_ss[static_cast<std::size_t>(b)] > 1e-6 || d_ff[static_cast<std::size_t>(b)] > 1e-6)
            ctx->area += br.area_per_ss;
    }

    ctx->obj = -lp_compute_score(ctx->wns_ss, ctx->tns_ss, ctx->wns_ff, ctx->tns_ff, ctx->area,
                                 pb->wns_ss_ori, pb->tns_ss_ori, pb->wns_ff_ori, pb->tns_ff_ori,
                                 pb->area_ori);
}

void project_bounds(const LpProblem *pb, std::vector<double> *d_ss, std::vector<double> *d_ff)
{
    const int n = static_cast<int>(pb->branches.size());
    for (int i = 0; i < n; i++) {
        const LpBranch &b = pb->branches[static_cast<std::size_t>(i)];
        auto &ss = (*d_ss)[static_cast<std::size_t>(i)];
        auto &ff = (*d_ff)[static_cast<std::size_t>(i)];
        ss = std::clamp(ss, b.d_ss_min, b.d_ss_max);
        ff = std::clamp(ff, b.d_ff_min, b.d_ff_max);
    }
}

void init_from_design(const LpProblem *pb, const PdDesign *d, std::vector<double> *d_ss,
                      std::vector<double> *d_ff)
{
    const int n = static_cast<int>(pb->branches.size());
    for (int b = 0; b < n; b++) {
        const LpBranch &br = pb->branches[static_cast<std::size_t>(b)];
        if (br.kind == LpBranchKind::ExistingBuf && br.cell_idx >= 0) {
            const PdCell *c = &d->cells[br.cell_idx];
            (*d_ss)[static_cast<std::size_t>(b)] = lp_eval_branch_delay_ss(d, c, br.fanout);
            (*d_ff)[static_cast<std::size_t>(b)] = lp_eval_branch_delay_ff(d, c, br.fanout);
        } else {
            (*d_ss)[static_cast<std::size_t>(b)] = 0.0;
            (*d_ff)[static_cast<std::size_t>(b)] = 0.0;
        }
    }
}

} // namespace

int lp_solve_mo_init(LpProblem *pb, const PdDesign *d, LpSolution *sol, double time_limit_sec,
                     char * /*err*/, std::size_t /*err_sz*/)
{
    sol->clear();
    LpPgCtx ctx;
    const int n_br = static_cast<int>(pb->branches.size());
    const double eps = 1e-6;
    const double step = 0.004;

    if (!build_pg_ctx(pb, d, &ctx))
        return -1;

    std::vector<double> d_ss(static_cast<std::size_t>(n_br));
    std::vector<double> d_ff(static_cast<std::size_t>(n_br));
    std::vector<double> g_ss(static_cast<std::size_t>(n_br), 0.0);
    std::vector<double> g_ff(static_cast<std::size_t>(n_br), 0.0);
    std::vector<double> best_ss(static_cast<std::size_t>(n_br));
    std::vector<double> best_ff(static_cast<std::size_t>(n_br));

    init_from_design(pb, d, &d_ss, &d_ff);
    project_bounds(pb, &d_ss, &d_ff);
    eval_solution(pb, d, d_ss, d_ff, &ctx);
    best_ss = d_ss;
    best_ff = d_ff;
    double best_obj = ctx.obj;

    const Clock::time_point t0 = Clock::now();
    const int n_paths = static_cast<int>(pb->path_ids.size());

    for (int iter = 0; elapsed_sec(t0) < time_limit_sec; iter++) {
        std::fill(g_ss.begin(), g_ss.end(), 0.0);
        std::fill(g_ff.begin(), g_ff.end(), 0.0);

        for (int p = 0; p < n_paths; p++) {
            const double ss_sl = ctx.slack_ss[static_cast<std::size_t>(p)];
            const double ff_sl = ctx.slack_ff[static_cast<std::size_t>(p)];

            double gw_ss = 0.0, gt_ss = 0.0, gw_ff = 0.0, gt_ff = 0.0;

            if (ss_sl < 0.0) {
                const double v = -ss_sl;
                gt_ss = -v / (std::fabs(pb->tns_ss_ori) + eps);
                if (ss_sl <= ctx.wns_ss + 1e-9)
                    gw_ss = -v / (std::fabs(pb->wns_ss_ori) + eps);
            }
            if (ff_sl < 0.0) {
                const double v = -ff_sl;
                gt_ff = -v / (std::fabs(pb->tns_ff_ori) + eps);
                if (ff_sl <= ctx.wns_ff + 1e-9)
                    gw_ff = -v / (std::fabs(pb->wns_ff_ori) + eps);
            }

            const int li = ctx.launch_ff[static_cast<std::size_t>(p)];
            const int ci = ctx.capture_ff[static_cast<std::size_t>(p)];

            if (ci >= 0) {
                for (int k = ctx.ff_path_off[static_cast<std::size_t>(ci)];
                     k < ctx.ff_path_off[static_cast<std::size_t>(ci + 1)]; k++) {
                    const int b = ctx.ff_path_br[static_cast<std::size_t>(k)];
                    g_ss[static_cast<std::size_t>(b)] += gw_ss + gt_ss;
                    g_ff[static_cast<std::size_t>(b)] += gw_ff * 0.5 + gt_ff * 0.5;
                }
            }
            if (li >= 0) {
                for (int k = ctx.ff_path_off[static_cast<std::size_t>(li)];
                     k < ctx.ff_path_off[static_cast<std::size_t>(li + 1)]; k++) {
                    const int b = ctx.ff_path_br[static_cast<std::size_t>(k)];
                    g_ss[static_cast<std::size_t>(b)] -= gw_ss + gt_ss;
                    g_ff[static_cast<std::size_t>(b)] -= gw_ff + gt_ff;
                }
            }
        }

        for (int b = 0; b < n_br; b++) {
            d_ss[static_cast<std::size_t>(b)] -= step * g_ss[static_cast<std::size_t>(b)];
            d_ff[static_cast<std::size_t>(b)] -= step * g_ff[static_cast<std::size_t>(b)];
        }
        project_bounds(pb, &d_ss, &d_ff);
        eval_solution(pb, d, d_ss, d_ff, &ctx);

        if (ctx.obj < best_obj) {
            best_obj = ctx.obj;
            best_ss = d_ss;
            best_ff = d_ff;
        }
    }

    sol->d_ss = best_ss;
    sol->d_ff = best_ff;
    sol->status = elapsed_sec(t0) >= time_limit_sec - 1e-6 ? 1 : 0;
    sol->solver_name = "mo_projected_gradient";
    return 0;
}

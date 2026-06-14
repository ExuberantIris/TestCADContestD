#include "sa_eval.hpp"

#include "lp_score.hpp"
#include "lp_types.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_map>

static double branch_area_dp(const LpBranch &br, double d_ss, double d_ff, const LpBufferChainDp *dp_ss,
                              const LpBufferChainDp *dp_ff)
{
    if (br.kind == LpBranchKind::Insertable) {
        if (d_ss < 1e-9 && d_ff < 1e-9)
            return 0.0;
        const int fo = 1;
        const auto &ess = dp_ss->lookup(fo, d_ss);
        const auto &eff = dp_ff->lookup(fo, d_ff);
        double a = 0.0;
        if (ess.reachable)
            a = std::max(a, ess.area);
        if (eff.reachable)
            a = std::max(a, eff.area);
        return a;
    }
    const auto &ess = dp_ss->lookup(br.fanout, d_ss);
    const auto &eff = dp_ff->lookup(br.fanout, d_ff);
    double a = 0.0;
    if (ess.reachable)
        a = std::max(a, ess.area);
    if (eff.reachable)
        a = std::max(a, eff.area);
    return a > 0.0 ? a : br.area_per_ss;
}

bool sa_build_ctx(const LpProblem *pb, const PdDesign *d, SaPgCtx *ctx)
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

    std::unordered_map<long long, int> edge_branch;
    edge_branch.reserve(static_cast<std::size_t>(n_br * 2));
    for (int b = 0; b < n_br; b++) {
        const LpBranch &br = pb->branches[static_cast<std::size_t>(b)];
        const long long key =
            (static_cast<long long>(br.parent_node) << 32) |
            static_cast<unsigned int>(br.child_node);
        edge_branch[key] = b;
    }

    ctx->ff_path_br.clear();
    ctx->ff_path_off.assign(static_cast<std::size_t>(n_ff + 1), 0);
    int off = 0;
    for (int f = 0; f < n_ff; f++) {
        ctx->ff_path_off[static_cast<std::size_t>(f)] = off;
        int cur = pb->ff_node_ids[static_cast<std::size_t>(f)];
        int steps = 0;
        while (cur >= 0 && steps++ < d->n_nodes) {
            const int parent = d->nodes[cur].parent;
            if (parent < 0)
                break;
            const long long key =
                (static_cast<long long>(parent) << 32) | static_cast<unsigned int>(cur);
            const auto it = edge_branch.find(key);
            if (it != edge_branch.end()) {
                ctx->ff_path_br.push_back(it->second);
                off++;
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

void sa_eval_state(const LpProblem *pb, const PdDesign *d, const std::vector<double> &d_ss,
                   const std::vector<double> &d_ff, const LpBufferChainDp *dp_ss,
                   const LpBufferChainDp *dp_ff, SaPgCtx *ctx)
{
    const int n_ff = static_cast<int>(pb->ff_node_ids.size());
    const int n_paths = static_cast<int>(pb->path_ids.size());
    const int n_br = static_cast<int>(pb->branches.size());

    if (static_cast<int>(ctx->T_ss.size()) != n_ff) {
        ctx->T_ss.assign(static_cast<std::size_t>(n_ff), 0.0);
        ctx->T_ff.assign(static_cast<std::size_t>(n_ff), 0.0);
    }
    if (static_cast<int>(ctx->slack_ss.size()) != n_paths) {
        ctx->slack_ss.assign(static_cast<std::size_t>(n_paths), 0.0);
        ctx->slack_ff.assign(static_cast<std::size_t>(n_paths), 0.0);
    }

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
        ctx->area += branch_area_dp(br, d_ss[static_cast<std::size_t>(b)],
                                    d_ff[static_cast<std::size_t>(b)], dp_ss, dp_ff);
    }

    ctx->score = lp_compute_score(ctx->wns_ss, ctx->tns_ss, ctx->wns_ff, ctx->tns_ff, ctx->area,
                                  pb->wns_ss_ori, pb->tns_ss_ori, pb->wns_ff_ori, pb->tns_ff_ori,
                                  pb->area_ori);
}

static void subsample_delays(std::vector<double> *delays, std::size_t max_n = 32)
{
    if (delays->size() <= max_n)
        return;
    std::vector<double> out;
    out.reserve(max_n);
    for (std::size_t i = 0; i < max_n; i++) {
        const std::size_t idx = i * (delays->size() - 1) / (max_n - 1);
        out.push_back((*delays)[idx]);
    }
    *delays = std::move(out);
}

void sa_build_branch_opts(const LpProblem *pb, const PdDesign *d, std::vector<BranchDpOpts> *opts)
{
    opts->resize(pb->branches.size());
    for (std::size_t b = 0; b < pb->branches.size(); b++) {
        const LpBranch &br = pb->branches[b];
        const int fo = br.kind == LpBranchKind::Insertable ? 1 : br.fanout;
        BranchDpOpts &o = (*opts)[b];

        o.ss_delays.push_back(0.0);
        o.ff_delays.push_back(0.0);
        for (int ci = 0; ci < d->n_cells; ci++) {
            const PdCell *c = &d->cells[ci];
            if (fo > c->max_fanout)
                continue;
            const double s = lp_eval_branch_delay_ss(d, c, fo);
            const double f = lp_eval_branch_delay_ff(d, c, fo);
            if (br.kind == LpBranchKind::ExistingBuf) {
                if (s >= br.d_ss_min - 1e-9 && s <= br.d_ss_max + 1e-9)
                    o.ss_delays.push_back(s);
                if (f >= br.d_ff_min - 1e-9 && f <= br.d_ff_max + 1e-9)
                    o.ff_delays.push_back(f);
            } else {
                if (s > 1e-9)
                    o.ss_delays.push_back(s);
                if (f > 1e-9)
                    o.ff_delays.push_back(f);
            }
        }

        if (br.kind == LpBranchKind::ExistingBuf) {
            o.ss_delays.push_back(br.d_ss_min);
            o.ss_delays.push_back(br.d_ss_max);
            o.ff_delays.push_back(br.d_ff_min);
            o.ff_delays.push_back(br.d_ff_max);
        }

        subsample_delays(&o.ss_delays);
        subsample_delays(&o.ff_delays);
    }
}

void sa_init_from_design(const LpProblem *pb, const PdDesign *d, const std::vector<BranchDpOpts> &opts,
                         std::vector<double> *d_ss, std::vector<double> *d_ff)
{
    const int n = static_cast<int>(pb->branches.size());
    d_ss->assign(static_cast<std::size_t>(n), 0.0);
    d_ff->assign(static_cast<std::size_t>(n), 0.0);
    for (int b = 0; b < n; b++) {
        const LpBranch &br = pb->branches[static_cast<std::size_t>(b)];
        double ss = 0.0, ff = 0.0;
        if (br.kind == LpBranchKind::ExistingBuf && br.cell_idx >= 0) {
            const PdCell *c = &d->cells[br.cell_idx];
            ss = lp_eval_branch_delay_ss(d, c, br.fanout);
            ff = lp_eval_branch_delay_ff(d, c, br.fanout);
        }
        const auto &o = opts[static_cast<std::size_t>(b)];
        auto nearest = [](double v, const std::vector<double> &lst) {
            double best = lst.empty() ? 0.0 : lst[0];
            double err = std::fabs(v - best);
            for (double x : lst) {
                const double e = std::fabs(v - x);
                if (e < err) {
                    err = e;
                    best = x;
                }
            }
            return best;
        };
        (*d_ss)[static_cast<std::size_t>(b)] = nearest(ss, o.ss_delays);
        (*d_ff)[static_cast<std::size_t>(b)] = nearest(ff, o.ff_delays);
    }
}

double sa_path_violation_weight(const SaPgCtx &ctx, int path_idx)
{
    double w = 0.0;
    const double ss = ctx.slack_ss[static_cast<std::size_t>(path_idx)];
    const double ff = ctx.slack_ff[static_cast<std::size_t>(path_idx)];
    if (ss < 0.0)
        w += -ss;
    if (ff < 0.0)
        w += -ff;
    return w;
}

void sa_path_branches_launch(const SaPgCtx &ctx, int path_idx, std::vector<int> *branches)
{
    branches->clear();
    const int li = ctx.launch_ff[static_cast<std::size_t>(path_idx)];
    if (li < 0)
        return;
    for (int k = ctx.ff_path_off[static_cast<std::size_t>(li)];
         k < ctx.ff_path_off[static_cast<std::size_t>(li + 1)]; k++)
        branches->push_back(ctx.ff_path_br[static_cast<std::size_t>(k)]);
}

void sa_path_branches_capture(const SaPgCtx &ctx, int path_idx, std::vector<int> *branches)
{
    branches->clear();
    const int ci = ctx.capture_ff[static_cast<std::size_t>(path_idx)];
    if (ci < 0)
        return;
    for (int k = ctx.ff_path_off[static_cast<std::size_t>(ci)];
         k < ctx.ff_path_off[static_cast<std::size_t>(ci + 1)]; k++)
        branches->push_back(ctx.ff_path_br[static_cast<std::size_t>(k)]);
}

void sa_branch_weights(const LpProblem *pb, const SaPgCtx &ctx, std::vector<double> *weights)
{
    const int n_br = static_cast<int>(pb->branches.size());
    const int n_paths = static_cast<int>(pb->path_ids.size());
    weights->assign(static_cast<std::size_t>(n_br), 1.0);

    for (int p = 0; p < n_paths; p++) {
        if (ctx.slack_ss[static_cast<std::size_t>(p)] >= 0.0 &&
            ctx.slack_ff[static_cast<std::size_t>(p)] >= 0.0)
            continue;

        const int li = ctx.launch_ff[static_cast<std::size_t>(p)];
        const int ci = ctx.capture_ff[static_cast<std::size_t>(p)];
        if (ci >= 0) {
            for (int k = ctx.ff_path_off[static_cast<std::size_t>(ci)];
                 k < ctx.ff_path_off[static_cast<std::size_t>(ci + 1)]; k++)
                (*weights)[static_cast<std::size_t>(ctx.ff_path_br[static_cast<std::size_t>(k)])] += 2.0;
        }
        if (li >= 0) {
            for (int k = ctx.ff_path_off[static_cast<std::size_t>(li)];
                 k < ctx.ff_path_off[static_cast<std::size_t>(li + 1)]; k++)
                (*weights)[static_cast<std::size_t>(ctx.ff_path_br[static_cast<std::size_t>(k)])] += 2.0;
        }
    }
}

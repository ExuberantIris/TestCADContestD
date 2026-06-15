#include "setup_lp_solve.hpp"

#include "lp_score.hpp"
#include "pd_clock.h"
#include "pd_output.h"
#include "sa_config.hpp"
#include "sa_eval.hpp"
#include "sa_params.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <queue>
#include <random>
#include <utility>
#include <vector>

namespace {

using Clock = std::chrono::steady_clock;

struct DgEdge {
    int from = 0;
    int to = 0;
    double weight = 0.0;
};

double elapsed_sec(const Clock::time_point &t0)
{
    return std::chrono::duration<double>(Clock::now() - t0).count();
}

int find_root(const PdDesign *d)
{
    for (int i = 0; i < d->n_nodes; i++) {
        if (d->nodes[i].parent < 0)
            return i;
    }
    return 0;
}

int branch_index(const LpProblem *pb, int parent, int child)
{
    for (std::size_t b = 0; b < pb->branches.size(); b++) {
        const LpBranch &br = pb->branches[b];
        if (br.parent_node == parent && br.child_node == child)
            return static_cast<int>(b);
    }
    return -1;
}

void clamp_branch_target(const LpBranch &br, double *target)
{
    *target = std::max(br.d_ss_min, std::min(br.d_ss_max, *target));
}

double path_required_skew(const PdDesign *d, const PdPath *path)
{
    return path->data_ss + d->t_setup - d->clock_period;
}

std::vector<double> build_ori_ff_clk_delay(const PdDesign *d)
{
    std::vector<double> ori(static_cast<std::size_t>(d->n_nodes), 0.0);
    for (int i = 0; i < d->n_nodes; i++) {
        if (d->nodes[i].kind == PD_NODE_FF)
            ori[static_cast<std::size_t>(i)] = d->nodes[i].d_clk_ss;
    }
    return ori;
}

double topo_node_key(const PdDesign *d, const std::vector<double> &ori_ff_clk, int node_id)
{
    if (d->nodes[node_id].kind == PD_NODE_FF)
        return ori_ff_clk[static_cast<std::size_t>(node_id)];
    return -1e30;
}

double subtree_ori_ff_key(const PdDesign *d, const std::vector<double> &ori_ff_clk, int node_id)
{
    const PdNode *n = &d->nodes[node_id];
    if (n->kind == PD_NODE_FF)
        return ori_ff_clk[static_cast<std::size_t>(node_id)];

    double best = 1e30;
    for (int i = 0; i < n->nchildren; i++)
        best = std::min(best, subtree_ori_ff_key(d, ori_ff_clk, n->children[i]));
    if (best >= 1e29)
        return 0.0;
    return best;
}

std::vector<int> topo_sort_pseudo_dag(int n_nodes, const PdDesign *d,
                                      const std::vector<double> &ori_ff_clk,
                                      const std::vector<DgEdge> &path_edges,
                                      const std::vector<std::vector<std::pair<int, double>>> &out)
{
    std::vector<int> indeg(static_cast<std::size_t>(n_nodes), 0);
    for (const DgEdge &e : path_edges)
        indeg[static_cast<std::size_t>(e.to)]++;

    std::vector<char> in_order(static_cast<std::size_t>(n_nodes), 0);
    std::vector<int> order;
    order.reserve(static_cast<std::size_t>(n_nodes));

    std::vector<int> ready;
    ready.reserve(static_cast<std::size_t>(n_nodes));
    for (int i = 0; i < n_nodes; i++) {
        if (indeg[static_cast<std::size_t>(i)] == 0)
            ready.push_back(i);
    }

    while (!ready.empty()) {
        std::sort(ready.begin(), ready.end(), [&](int a, int b) {
            const double ka = topo_node_key(d, ori_ff_clk, a);
            const double kb = topo_node_key(d, ori_ff_clk, b);
            if (ka != kb)
                return ka < kb;
            return a < b;
        });

        const int u = ready.front();
        ready.erase(ready.begin());
        order.push_back(u);
        in_order[static_cast<std::size_t>(u)] = 1;

        for (const auto &nv : out[static_cast<std::size_t>(u)]) {
            const int v = nv.first;
            if (--indeg[static_cast<std::size_t>(v)] == 0)
                ready.push_back(v);
        }
    }

    for (int i = 0; i < n_nodes; i++) {
        if (!in_order[static_cast<std::size_t>(i)])
            order.push_back(i);
    }
    return order;
}

void relax_path_arrivals_topo(const std::vector<int> &order,
                              const std::vector<std::vector<std::pair<int, double>>> &out,
                              int root, int n_nodes, std::vector<double> *arrival)
{
    arrival->assign(static_cast<std::size_t>(n_nodes), 0.0);
    (*arrival)[static_cast<std::size_t>(root)] = 0.0;

    for (int u : order) {
        for (const auto &nv : out[static_cast<std::size_t>(u)]) {
            const int v = nv.first;
            const double q = nv.second;
            const double cand = (*arrival)[static_cast<std::size_t>(u)] + q;
            if (cand > (*arrival)[static_cast<std::size_t>(v)] + 1e-12)
                (*arrival)[static_cast<std::size_t>(v)] = cand;
        }
    }
}

void build_timing_path_dag(const PdDesign *d, const LpProblem *pb, std::vector<DgEdge> *path_edges,
                           std::vector<std::vector<std::pair<int, double>>> *out_adj)
{
    const int n_nodes = d->n_nodes;
    const int n_paths = static_cast<int>(pb->path_ids.size());
    path_edges->clear();
    out_adj->assign(static_cast<std::size_t>(n_nodes), {});

    for (int pi = 0; pi < n_paths; pi++) {
        const PdPath *path = &d->paths[pb->path_ids[static_cast<std::size_t>(pi)]];
        if (path->launch_id < 0 || path->capture_id < 0)
            continue;

        DgEdge e;
        e.from = path->launch_id;
        e.to = path->capture_id;
        e.weight = path_required_skew(d, path);
        path_edges->push_back(e);
        (*out_adj)[static_cast<std::size_t>(e.from)].push_back({e.to, e.weight});
    }
}

bool compute_ideal_ff_arrival(const PdDesign *d, const LpProblem *pb,
                              const std::vector<double> &ori_ff_clk, std::vector<double> *ideal)
{
    const int root = find_root(d);
    const int n_nodes = d->n_nodes;

    std::vector<DgEdge> path_edges;
    std::vector<std::vector<std::pair<int, double>>> out_adj;
    build_timing_path_dag(d, pb, &path_edges, &out_adj);

    const std::vector<int> order = topo_sort_pseudo_dag(n_nodes, d, ori_ff_clk, path_edges, out_adj);
    relax_path_arrivals_topo(order, out_adj, root, n_nodes, ideal);
    return true;
}

/**
 * Bottom-up segment tree over the clock tree:
 * each child returns max ideal arrival among FFs in its subtree.
 * For siblings a,b with a>b: parent edge to a gets 0, edge to b gets (a-b).
 */
double build_seg_tree_targets(const PdDesign *d, const LpProblem *pb,
                              const std::vector<double> &ideal_arrival,
                              const std::vector<double> &ori_ff_clk,
                              std::vector<double> *target_d_ss, int node_id)
{
    const PdNode *n = &d->nodes[node_id];

    if (n->kind == PD_NODE_FF)
        return ideal_arrival[static_cast<std::size_t>(node_id)];

    std::vector<int> children;
    children.reserve(static_cast<std::size_t>(n->nchildren));
    for (int i = 0; i < n->nchildren; i++)
        children.push_back(n->children[i]);

    std::sort(children.begin(), children.end(), [&](int a, int b) {
        const double ka = subtree_ori_ff_key(d, ori_ff_clk, a);
        const double kb = subtree_ori_ff_key(d, ori_ff_clk, b);
        if (ka != kb)
            return ka < kb;
        return a < b;
    });

    std::vector<double> child_req;
    std::vector<int> child_ids;
    child_req.reserve(children.size());
    child_ids.reserve(children.size());

    for (int child : children) {
        child_req.push_back(
            build_seg_tree_targets(d, pb, ideal_arrival, ori_ff_clk, target_d_ss, child));
        child_ids.push_back(child);
    }

    if (child_req.empty())
        return 0.0;

    const double subtree_max = *std::max_element(child_req.begin(), child_req.end());

    for (std::size_t i = 0; i < child_ids.size(); i++) {
        const int child = child_ids[i];
        const int b = branch_index(pb, node_id, child);
        if (b < 0)
            continue;

        double edge_target = subtree_max - child_req[i];
        clamp_branch_target(pb->branches[static_cast<std::size_t>(b)], &edge_target);
        (*target_d_ss)[static_cast<std::size_t>(b)] = edge_target;
    }

    return subtree_max;
}

void build_all_seg_tree_targets(const PdDesign *d, const LpProblem *pb,
                                const std::vector<double> &ideal_arrival,
                                const std::vector<double> &ori_ff_clk,
                                std::vector<double> *target_d_ss)
{
    target_d_ss->assign(pb->branches.size(), 0.0);
    build_seg_tree_targets(d, pb, ideal_arrival, ori_ff_clk, target_d_ss, find_root(d));
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

double snap_toward_target(double cur, double target, const std::vector<double> &lst)
{
    if (target > cur + 1e-9)
        return next_higher(cur, lst);
    if (target < cur - 1e-9)
        return next_lower(cur, lst);
    return cur;
}

double weighted_timing_score(const LpProblem *pb, const SaPgCtx &ctx, const LpScoreWeights &wt)
{
    return lp_compute_weighted_score(ctx.wns_ss, ctx.tns_ss, ctx.wns_ff, ctx.tns_ff, ctx.area,
                                   pb->wns_ss_ori, pb->tns_ss_ori, pb->wns_ff_ori, pb->tns_ff_ori,
                                   pb->area_ori, wt);
}

void filter_existing_buf_branches(const LpProblem *pb, const std::vector<int> &raw,
                                  const std::vector<int> &branch_stall, int stall_limit,
                                  std::vector<int> *out)
{
    out->clear();
    for (int b : raw) {
        if (pb->branches[static_cast<std::size_t>(b)].kind != LpBranchKind::ExistingBuf)
            continue;
        if (branch_stall[static_cast<std::size_t>(b)] >= stall_limit)
            continue;
        out->push_back(b);
    }
}

void build_weighted_violating_paths(const SaPgCtx &ctx, const LpScoreWeights &wt,
                                    std::vector<int> *pool)
{
    pool->clear();
    std::vector<std::pair<double, int>> ranked;
    ranked.reserve(ctx.slack_ss.size());

    for (std::size_t p = 0; p < ctx.slack_ss.size(); p++) {
        double w = 0.0;
        if (ctx.slack_ss[p] < 0.0)
            w += wt.a * (-ctx.slack_ss[p]);
        if (ctx.slack_ff[p] < 0.0)
            w += wt.b * (-ctx.slack_ff[p]);
        if (w > 1e-12)
            ranked.push_back({w, static_cast<int>(p)});
    }

    std::sort(ranked.begin(), ranked.end(),
              [](const std::pair<double, int> &a, const std::pair<double, int> &b) {
                  if (a.first != b.first)
                      return a.first > b.first;
                  return a.second < b.second;
              });

    const int top_n = std::min(SaConfig::kPhase2TopPathPool, static_cast<int>(ranked.size()));
    pool->reserve(static_cast<std::size_t>(top_n));
    for (int i = 0; i < top_n; i++)
        pool->push_back(ranked[static_cast<std::size_t>(i)].second);
}

bool try_path_directed_move(int path_idx, const SaPgCtx &ctx, const LpProblem *pb,
                            const std::vector<BranchDpOpts> &opts,
                            const std::vector<int> &branch_stall, int stall_limit,
                            std::mt19937 &rng, std::vector<double> *trial_ss, int *moved_branch)
{
    const double ss_slack = ctx.slack_ss[static_cast<std::size_t>(path_idx)];
    const double ff_slack = ctx.slack_ff[static_cast<std::size_t>(path_idx)];
    const bool fix_setup = ss_slack < -1e-12;
    const bool fix_hold = ff_slack < -1e-12;
    if (!fix_setup && !fix_hold)
        return false;

    bool use_setup = fix_setup;
    if (fix_setup && fix_hold)
        use_setup = std::uniform_real_distribution<double>(0.0, 1.0)(rng) < 0.75;

    std::vector<int> raw;
    if (use_setup)
        sa_path_branches_capture(ctx, path_idx, &raw);
    else
        sa_path_branches_launch(ctx, path_idx, &raw);

    std::vector<int> eligible;
    filter_existing_buf_branches(pb, raw, branch_stall, stall_limit, &eligible);
    if (eligible.empty())
        return false;

    const int b = eligible[static_cast<std::size_t>(std::uniform_int_distribution<int>(
        0, static_cast<int>(eligible.size()) - 1)(rng))];
    const auto &o = opts[static_cast<std::size_t>(b)];
    const double cur = (*trial_ss)[static_cast<std::size_t>(b)];
    double neu = cur;

    if (use_setup) {
        if (o.ss_delays.size() <= 1)
            return false;
        neu = next_higher(cur, o.ss_delays);
    } else {
        if (o.ss_delays.size() <= 1)
            return false;
        neu = next_lower(cur, o.ss_delays);
    }

    if (std::fabs(neu - cur) < 1e-12)
        return false;

    (*trial_ss)[static_cast<std::size_t>(b)] = neu;
    *moved_branch = b;
    return true;
}

bool try_area_recovery_move(const LpProblem *pb, const std::vector<BranchDpOpts> &opts,
                            const std::vector<double> &cur_ss, const std::vector<int> &branch_stall,
                            int stall_limit, std::mt19937 &rng, std::vector<double> *trial_ss,
                            int *moved_branch)
{
    std::vector<std::pair<double, int>> ranked;
    const int n_br = static_cast<int>(pb->branches.size());
    ranked.reserve(static_cast<std::size_t>(n_br));

    for (int b = 0; b < n_br; b++) {
        if (pb->branches[static_cast<std::size_t>(b)].kind != LpBranchKind::ExistingBuf)
            continue;
        if (branch_stall[static_cast<std::size_t>(b)] >= stall_limit)
            continue;
        ranked.push_back({cur_ss[static_cast<std::size_t>(b)], b});
    }
    if (ranked.empty())
        return false;

    std::sort(ranked.begin(), ranked.end(),
              [](const std::pair<double, int> &a, const std::pair<double, int> &b) {
                  if (a.first != b.first)
                      return a.first > b.first;
                  return a.second < b.second;
              });

    const int top_n =
        std::min(SaConfig::kPhase2AreaBranchPool, static_cast<int>(ranked.size()));
    const int pick = ranked[static_cast<std::size_t>(
                             std::uniform_int_distribution<int>(0, top_n - 1)(rng))]
                         .second;
    const auto &o = opts[static_cast<std::size_t>(pick)];
    if (o.ss_delays.size() <= 1)
        return false;

    const double cur = cur_ss[static_cast<std::size_t>(pick)];
    const double neu = next_lower(cur, o.ss_delays);
    if (std::fabs(neu - cur) < 1e-12)
        return false;

    (*trial_ss)[static_cast<std::size_t>(pick)] = neu;
    *moved_branch = pick;
    return true;
}

bool try_gap_refine_move(const std::vector<double> &target_d_ss, const std::vector<BranchDpOpts> &opts,
                         const LpProblem *pb, const std::vector<double> &cur_ss,
                         const std::vector<int> &branch_stall, int stall_limit, std::mt19937 &rng,
                         std::vector<double> *trial_ss, int *moved_branch)
{
    std::vector<std::pair<double, int>> ranked;
    const int n_br = static_cast<int>(pb->branches.size());
    for (int b = 0; b < n_br; b++) {
        if (pb->branches[static_cast<std::size_t>(b)].kind != LpBranchKind::ExistingBuf)
            continue;
        if (branch_stall[static_cast<std::size_t>(b)] >= stall_limit)
            continue;
        const double gap =
            target_d_ss[static_cast<std::size_t>(b)] - cur_ss[static_cast<std::size_t>(b)];
        ranked.push_back({std::fabs(gap), b});
    }
    if (ranked.empty())
        return false;

    std::sort(ranked.begin(), ranked.end(),
              [](const std::pair<double, int> &a, const std::pair<double, int> &b) {
                  if (a.first != b.first)
                      return a.first > b.first;
                  return a.second < b.second;
              });

    const int top_k =
        std::min(SaConfig::kSaLeafFfPickCount, static_cast<int>(ranked.size()));
    const int b = ranked[static_cast<std::size_t>(
                            std::uniform_int_distribution<int>(0, top_k - 1)(rng))]
                        .second;
    const double target = target_d_ss[static_cast<std::size_t>(b)];
    const double neu =
        snap_toward_target(cur_ss[static_cast<std::size_t>(b)], target,
                           opts[static_cast<std::size_t>(b)].ss_delays);
    if (std::fabs(neu - cur_ss[static_cast<std::size_t>(b)]) < 1e-12)
        return false;

    (*trial_ss)[static_cast<std::size_t>(b)] = neu;
    *moved_branch = b;
    return true;
}

void propagate_tree_arrivals(const PdDesign *d, const LpProblem *pb, const std::vector<double> &d_ss,
                             std::vector<double> *arrival)
{
    const int root = find_root(d);
    arrival->assign(static_cast<std::size_t>(d->n_nodes), 0.0);

    std::queue<int> q;
    q.push(root);
    (*arrival)[static_cast<std::size_t>(root)] = 0.0;

    while (!q.empty()) {
        const int u = q.front();
        q.pop();
        const PdNode *nu = &d->nodes[u];
        for (int i = 0; i < nu->nchildren; i++) {
            const int v = nu->children[i];
            double delay = 0.0;
            const int b = branch_index(pb, u, v);
            if (b >= 0)
                delay = d_ss[static_cast<std::size_t>(b)];
            (*arrival)[static_cast<std::size_t>(v)] =
                (*arrival)[static_cast<std::size_t>(u)] + delay;
            q.push(v);
        }
    }
}

const char *node_kind_str(PdNodeKind kind)
{
    switch (kind) {
    case PD_NODE_ROOT:
        return "ROOT";
    case PD_NODE_BUF:
        return "BUF";
    case PD_NODE_FF:
        return "FF";
    default:
        return "?";
    }
}

double real_buf_ss_delay(const PdDesign *d, int buf_node_id)
{
    const PdNode *n = &d->nodes[buf_node_id];
    if (n->kind != PD_NODE_BUF || n->cell_idx < 0)
        return 0.0;
    const PdCell *c = &d->cells[n->cell_idx];
    int fanout = n->fanout > 0 ? n->fanout : 1;
    fanout = std::max(1, std::min(fanout, c->max_fanout));
    return c->ss_delay[fanout - 1];
}

double real_incoming_edge_delay(const PdDesign *d, int node_id)
{
    const PdNode *n = &d->nodes[node_id];
    if (n->parent < 0)
        return 0.0;
    if (n->kind == PD_NODE_BUF)
        return real_buf_ss_delay(d, node_id);
    return 0.0;
}

void propagate_real_clock_arrivals(const PdDesign *d, std::vector<double> *arrival)
{
    const int root = find_root(d);
    arrival->assign(static_cast<std::size_t>(d->n_nodes), 0.0);

    std::queue<int> q;
    q.push(root);
    (*arrival)[static_cast<std::size_t>(root)] = 0.0;

    while (!q.empty()) {
        const int u = q.front();
        q.pop();
        const PdNode *nu = &d->nodes[u];
        for (int i = 0; i < nu->nchildren; i++) {
            const int v = nu->children[i];
            const PdNode *nv = &d->nodes[v];
            if (nv->kind == PD_NODE_FF)
                (*arrival)[static_cast<std::size_t>(v)] = nv->d_clk_ss;
            else
                (*arrival)[static_cast<std::size_t>(v)] =
                    (*arrival)[static_cast<std::size_t>(u)] + real_buf_ss_delay(d, v);
            q.push(v);
        }
    }
}

struct LogicalNode {
    int id = -1;
    int level = 0;
    int parent = -1;
    char name[PD_MAX_NAME * 2 + 8];
    char kind[16];
    char repr[PD_MAX_NAME * 2 + 16];
    bool in_clock = false;
    bool in_seg = false;
    double clock_in_edge = 0.0;
    double clock_delay = 0.0;
    double seg_in_edge = 0.0;
    double seg_delay = 0.0;
};

bool is_merged_ff_child(const PdDesign *d, int node_id)
{
    const PdNode *n = &d->nodes[node_id];
    if (n->kind != PD_NODE_FF || n->parent < 0)
        return false;
    return d->nodes[n->parent].nchildren == 1;
}

int logical_rep_id(const PdDesign *d, int node_id)
{
    if (is_merged_ff_child(d, node_id))
        return d->nodes[node_id].parent;
    return node_id;
}

double incoming_edge_delay(const PdDesign *d, const LpProblem *pb, int node_id,
                           const std::vector<double> &d_ss)
{
    const PdNode *n = &d->nodes[node_id];
    if (n->parent < 0)
        return 0.0;
    const int b = branch_index(pb, n->parent, node_id);
    if (b < 0)
        return 0.0;
    return d_ss[static_cast<std::size_t>(b)];
}

void build_logical_name(const PdDesign *d, int rep_id, char *out, std::size_t out_sz)
{
    const PdNode *rep = &d->nodes[rep_id];
    if (rep->nchildren == 1 && d->nodes[rep->children[0]].kind == PD_NODE_FF) {
        const PdNode *ff = &d->nodes[rep->children[0]];
        std::snprintf(out, out_sz, "%s+%s", rep->name, ff->name);
        return;
    }
    std::snprintf(out, out_sz, "%s", rep->name);
}

void build_logical_repr(const PdDesign *d, int rep_id, char *out, std::size_t out_sz)
{
    const PdNode *rep = &d->nodes[rep_id];
    if (rep->nchildren == 1 && d->nodes[rep->children[0]].kind == PD_NODE_FF) {
        const int ff = rep->children[0];
        std::snprintf(out, out_sz, "%d+%d", rep_id, ff);
        return;
    }
    std::snprintf(out, out_sz, "%d", rep_id);
}

void mark_logical_nodes(const PdDesign *d, const LpProblem *pb,
                        const std::vector<double> &clock_arrival,
                        const std::vector<double> &seg_arrival,
                        const std::vector<double> &seg_d_ss, std::vector<LogicalNode> *nodes)
{
    std::vector<char> seen(static_cast<std::size_t>(d->n_nodes), 0);

    for (int i = 0; i < d->n_nodes; i++) {
        if (is_merged_ff_child(d, i))
            continue;

        const int rep = logical_rep_id(d, i);
        if (seen[static_cast<std::size_t>(rep)])
            continue;
        seen[static_cast<std::size_t>(rep)] = 1;

        LogicalNode ln;
        ln.id = rep;
        ln.level = d->nodes[rep].level;
        ln.parent = d->nodes[rep].parent;
        if (ln.parent >= 0 && is_merged_ff_child(d, ln.parent))
            ln.parent = logical_rep_id(d, ln.parent);
        build_logical_name(d, rep, ln.name, sizeof(ln.name));
        build_logical_repr(d, rep, ln.repr, sizeof(ln.repr));

        if (d->nodes[rep].kind == PD_NODE_FF)
            std::snprintf(ln.kind, sizeof(ln.kind), "FF");
        else if (d->nodes[rep].nchildren == 1 &&
                 d->nodes[d->nodes[rep].children[0]].kind == PD_NODE_FF)
            std::snprintf(ln.kind, sizeof(ln.kind), "BUF+FF");
        else
            std::snprintf(ln.kind, sizeof(ln.kind), "%s", node_kind_str(d->nodes[rep].kind));

        ln.in_clock = true;
        ln.in_seg = true;

        if (d->nodes[rep].nchildren == 1 &&
            d->nodes[d->nodes[rep].children[0]].kind == PD_NODE_FF) {
            const int ff = d->nodes[rep].children[0];
            ln.clock_in_edge =
                real_incoming_edge_delay(d, rep) + real_incoming_edge_delay(d, ff);
            ln.seg_in_edge = incoming_edge_delay(d, pb, rep, seg_d_ss) +
                             incoming_edge_delay(d, pb, ff, seg_d_ss);
            ln.clock_delay = clock_arrival[static_cast<std::size_t>(ff)];
            ln.seg_delay = seg_arrival[static_cast<std::size_t>(ff)];
        } else {
            ln.clock_in_edge = real_incoming_edge_delay(d, rep);
            ln.seg_in_edge = incoming_edge_delay(d, pb, rep, seg_d_ss);
            ln.clock_delay = clock_arrival[static_cast<std::size_t>(rep)];
            ln.seg_delay = seg_arrival[static_cast<std::size_t>(rep)];
        }

        nodes->push_back(ln);
    }

    std::sort(nodes->begin(), nodes->end(),
              [](const LogicalNode &a, const LogicalNode &b) { return a.id < b.id; });
}

void write_delay_field(FILE *fp, bool present, double value)
{
    if (present)
        std::fprintf(fp, "\t%.6f", value);
    else
        std::fprintf(fp, "\t--");
}

} // namespace

int ff_ori_clk_delay_write(const PdDesign *d, const LpProblem *pb,
                           const std::vector<double> &ori_ff_clk, const char *out_dir, char *err,
                           std::size_t err_sz)
{
    if (!d || !pb || !out_dir) {
        if (err && err_sz > 0)
            std::snprintf(err, err_sz, "null argument");
        return -1;
    }

    char out_path[1024];
    if (pd_join_path(out_path, sizeof(out_path), out_dir, "ff_ori_clk_delay.txt") != 0) {
        if (err && err_sz > 0)
            std::snprintf(err, err_sz, "output path too long");
        return -1;
    }

    std::vector<DgEdge> path_edges;
    std::vector<std::vector<std::pair<int, double>>> out_adj;
    build_timing_path_dag(d, pb, &path_edges, &out_adj);

    const std::vector<int> order =
        topo_sort_pseudo_dag(d->n_nodes, d, ori_ff_clk, path_edges, out_adj);

    std::vector<int> topo_rank(static_cast<std::size_t>(d->n_nodes), -1);
    for (std::size_t i = 0; i < order.size(); i++)
        topo_rank[static_cast<std::size_t>(order[i])] = static_cast<int>(i);

    FILE *fp = std::fopen(out_path, "w");
    if (!fp) {
        if (err && err_sz > 0)
            std::snprintf(err, err_sz, "cannot open %s", out_path);
        return -1;
    }

    std::fprintf(fp, "# Original clock tree delay (SS) per FF before optimization\n");
    std::fprintf(fp, "# topo_rank: pseudo-DAG order (lower ori delay preferred when ties)\n");
    std::fprintf(fp, "ff_node_id\tff_name\tori_clk_delay\ttopo_rank\n");

    struct FfRow {
        int id = 0;
        const char *name = nullptr;
        double delay = 0.0;
        int rank = 0;
    };
    std::vector<FfRow> rows;
    rows.reserve(pb->ff_node_ids.size());

    for (int ff_id : pb->ff_node_ids) {
        FfRow row;
        row.id = ff_id;
        row.name = d->nodes[ff_id].name;
        row.delay = ori_ff_clk[static_cast<std::size_t>(ff_id)];
        row.rank = topo_rank[static_cast<std::size_t>(ff_id)];
        rows.push_back(row);
    }

    std::sort(rows.begin(), rows.end(),
              [](const FfRow &a, const FfRow &b) { return a.rank < b.rank; });

    for (const FfRow &row : rows)
        std::fprintf(fp, "%d\t%s\t%.6f\t%d\n", row.id, row.name, row.delay, row.rank);

    std::fclose(fp);
    std::printf("Wrote %s\n", out_path);
    return 0;
}

int seg_tree_write_outputs(const PdDesign *d, const LpProblem *pb,
                           const std::vector<double> &ori_ff_clk, const char *out_dir, char *err,
                           std::size_t err_sz)
{
    if (!d || !pb || !out_dir) {
        if (err && err_sz > 0)
            std::snprintf(err, err_sz, "null argument");
        return -1;
    }

    char buffer_path[1024];
    char compare_path[1024];
    if (pd_join_path(buffer_path, sizeof(buffer_path), out_dir, "buffer_clock.txt") != 0 ||
        pd_join_path(compare_path, sizeof(compare_path), out_dir, "delay_compare.txt") != 0) {
        if (err && err_sz > 0)
            std::snprintf(err, err_sz, "output path too long");
        return -1;
    }

    std::vector<double> ideal_arrival;
    if (!compute_ideal_ff_arrival(d, pb, ori_ff_clk, &ideal_arrival)) {
        if (err && err_sz > 0)
            std::snprintf(err, err_sz, "failed to compute ideal FF arrival");
        return -1;
    }

    std::vector<double> seg_d_ss;
    build_all_seg_tree_targets(d, pb, ideal_arrival, ori_ff_clk, &seg_d_ss);

    std::vector<double> clock_arrival;
    std::vector<double> seg_arrival;
    propagate_real_clock_arrivals(d, &clock_arrival);
    propagate_tree_arrivals(d, pb, seg_d_ss, &seg_arrival);

    std::vector<LogicalNode> logical;
    mark_logical_nodes(d, pb, clock_arrival, seg_arrival, seg_d_ss, &logical);

    FILE *bfp = std::fopen(buffer_path, "w");
    if (!bfp) {
        if (err && err_sz > 0)
            std::snprintf(err, err_sz, "cannot open %s", buffer_path);
        return -1;
    }

    std::fprintf(bfp, "# Segment tree (bottom-up target delays, SS corner)\n");
    std::fprintf(bfp, "# node_delay = cumulative arrival from root at logical node\n");
    std::fprintf(bfp, "# fanout==1 FF merged with parent as one logical node\n");
    std::fprintf(bfp,
                 "logical_id\tname\tkind\tlevel\tparent\trepr_nodes\t"
                 "seg_in_edge_delay\tseg_node_delay\n");

    for (const LogicalNode &ln : logical) {
        std::fprintf(bfp, "%d\t%s\t%s\t%d\t%d\t%s\t%.6f\t%.6f\n", ln.id, ln.name, ln.kind, ln.level,
                     ln.parent, ln.repr, ln.seg_in_edge, ln.seg_delay);
    }
    std::fclose(bfp);

    FILE *cfp = std::fopen(compare_path, "w");
    if (!cfp) {
        if (err && err_sz > 0)
            std::snprintf(err, err_sz, "cannot open %s", compare_path);
        return -1;
    }

    std::fprintf(cfp, "# Clock tree = real buffer cell delays (SS, after apply)\n");
    std::fprintf(cfp, "# Segment tree = bottom-up target branch delays\n");
    std::fprintf(cfp, "# *_in_edge = incoming buffer delay; *_node_delay = cumulative from root\n");
    std::fprintf(cfp, "# fanout==1 FF merged with parent; -- if node absent in that tree\n");
    std::fprintf(cfp,
                 "logical_id\tname\tkind\tlevel\trepr_nodes\t"
                 "clock_in_edge\tclock_node_delay\tseg_in_edge\tseg_node_delay\tdiff\n");

    for (const LogicalNode &ln : logical) {
        std::fprintf(cfp, "%d\t%s\t%s\t%d\t%s", ln.id, ln.name, ln.kind, ln.level, ln.repr);
        write_delay_field(cfp, ln.in_clock, ln.clock_in_edge);
        write_delay_field(cfp, ln.in_clock, ln.clock_delay);
        write_delay_field(cfp, ln.in_seg, ln.seg_in_edge);
        write_delay_field(cfp, ln.in_seg, ln.seg_delay);
        if (ln.in_clock && ln.in_seg)
            std::fprintf(cfp, "\t%.6f\n", ln.seg_delay - ln.clock_delay);
        else
            std::fprintf(cfp, "\t--\n");
    }
    std::fclose(cfp);

    std::printf("Wrote %s\n", buffer_path);
    std::printf("Wrote %s\n", compare_path);
    return 0;
}

int setup_longest_path_solve(LpProblem *pb, const PdDesign *d, const LpBufferChainDp *dp_ss,
                             const LpBufferChainDp *dp_ff, const LpSolution *initial,
                             double time_limit_sec, SaSolveResult *out, char *err,
                             std::size_t err_sz)
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
            std::snprintf(err, err_sz, "failed to build timing context");
        return -1;
    }
    (void)dp_ss;
    (void)dp_ff;
    (void)time_limit_sec;

    std::vector<double> ori_ff_clk = build_ori_ff_clk_delay(d);

    std::vector<double> ideal_arrival;
    if (!compute_ideal_ff_arrival(d, pb, ori_ff_clk, &ideal_arrival)) {
        if (err && err_sz > 0)
            std::snprintf(err, err_sz, "failed to compute ideal FF arrival");
        return -1;
    }

    std::vector<double> target_d_ss;
    build_all_seg_tree_targets(d, pb, ideal_arrival, ori_ff_clk, &target_d_ss);

    const Clock::time_point t0 = Clock::now();
    out->iterations = 1;
    out->elapsed_sec = elapsed_sec(t0);
    out->timed_out = 0;
    out->solution.d_ss = target_d_ss;
    out->solution.d_ff = initial->d_ff;
    out->solution.status = 0;
    out->solution.solver_name = "seg_tree_direct";

    return 0;
}

int seg_tree_sa_solve(LpProblem *pb, const PdDesign *d, const LpBufferChainDp *dp_ss,
                      const LpBufferChainDp *dp_ff, const LpSolution *initial,
                      const std::vector<double> &ori_ff_clk, const SaParams &params,
                      double time_limit_sec, SaSolveResult *out, char *err, std::size_t err_sz)
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
            std::snprintf(err, err_sz, "failed to build timing context");
        return -1;
    }

    std::vector<BranchDpOpts> opts;
    sa_build_branch_opts(pb, d, &opts);

    std::vector<double> ideal_arrival;
    if (!compute_ideal_ff_arrival(d, pb, ori_ff_clk, &ideal_arrival)) {
        if (err && err_sz > 0)
            std::snprintf(err, err_sz, "failed to compute ideal FF arrival");
        return -1;
    }

    std::vector<double> target_d_ss;
    build_all_seg_tree_targets(d, pb, ideal_arrival, ori_ff_clk, &target_d_ss);

    const int n_br = static_cast<int>(pb->branches.size());
    std::vector<double> cur_ss = initial->d_ss;
    std::vector<double> cur_ff = initial->d_ff;
    if (static_cast<int>(cur_ss.size()) != n_br)
        sa_init_from_design(pb, d, opts, &cur_ss, &cur_ff);

    sa_eval_state(pb, d, cur_ss, cur_ff, dp_ss, dp_ff, &ctx);
    const LpScoreWeights &wt = params.score_weights;

    double best_score = weighted_timing_score(pb, ctx, wt);
    std::vector<double> best_ss = cur_ss;
    std::vector<double> best_ff = cur_ff;

    std::vector<int> branch_stall(static_cast<std::size_t>(n_br), 0);
    std::mt19937 rng(static_cast<unsigned>(
        std::chrono::steady_clock::now().time_since_epoch().count()));
    std::uniform_real_distribution<double> uni01(0.0, 1.0);

    const Clock::time_point t0 = Clock::now();
    double temperature = SaConfig::kSaTemperatureInit;
    int no_improve_iters = 0;
    int stalled = 0;

    while (elapsed_sec(t0) < time_limit_sec) {
        out->iterations++;

        sa_eval_state(pb, d, cur_ss, cur_ff, dp_ss, dp_ff, &ctx);

        std::vector<int> viol_paths;
        build_weighted_violating_paths(ctx, wt, &viol_paths);

        std::vector<double> trial_ss = cur_ss;
        int moved_branch = -1;
        bool moved = false;

        if (!viol_paths.empty() &&
            std::uniform_real_distribution<double>(0.0, 1.0)(rng) < SaConfig::kPhase2PathMoveProb) {
            const int path_idx = viol_paths[static_cast<std::size_t>(
                std::uniform_int_distribution<int>(0, static_cast<int>(viol_paths.size()) - 1)(rng))];
            moved = try_path_directed_move(path_idx, ctx, pb, opts, branch_stall,
                                           SaConfig::kSaBranchNoImproveLimit, rng, &trial_ss,
                                           &moved_branch);
        }

        if (!moved)
            moved = try_area_recovery_move(pb, opts, cur_ss, branch_stall,
                                           SaConfig::kSaBranchNoImproveLimit, rng, &trial_ss,
                                           &moved_branch);

        if (!moved) {
            moved = try_gap_refine_move(target_d_ss, opts, pb, cur_ss, branch_stall,
                                        SaConfig::kSaBranchNoImproveLimit, rng, &trial_ss,
                                        &moved_branch);
        }

        if (!moved || moved_branch < 0) {
            no_improve_iters++;
            if (no_improve_iters >= params.no_improve_limit) {
                stalled = 1;
                break;
            }
            continue;
        }

        const double old_score = weighted_timing_score(pb, ctx, wt);
        SaPgCtx trial_ctx = ctx;
        sa_eval_state(pb, d, trial_ss, cur_ff, dp_ss, dp_ff, &trial_ctx);
        const double new_score = weighted_timing_score(pb, trial_ctx, wt);
        const double delta = new_score - old_score;
        const bool accept =
            delta > 0.0 ||
            (temperature > 1e-9 && uni01(rng) < std::exp(delta / temperature));

        bool improved_best = false;
        if (accept) {
            cur_ss = std::move(trial_ss);
            ctx = trial_ctx;
            if (new_score > best_score + 1e-12) {
                best_score = new_score;
                best_ss = cur_ss;
                best_ff = cur_ff;
                improved_best = true;
            }
        }

        if (improved_best)
            branch_stall[static_cast<std::size_t>(moved_branch)] = 0;
        else
            branch_stall[static_cast<std::size_t>(moved_branch)]++;

        if (improved_best) {
            no_improve_iters = 0;
        } else {
            no_improve_iters++;
            if (no_improve_iters >= params.no_improve_limit) {
                stalled = 1;
                break;
            }
        }

        temperature *= SaConfig::kSaTemperatureDecay;
        if (temperature < SaConfig::kSaTemperatureFloor)
            temperature = SaConfig::kSaTemperatureInit;
    }

    out->elapsed_sec = elapsed_sec(t0);
    out->timed_out = (out->elapsed_sec >= time_limit_sec - 1e-6) ? 1 : 0;
    out->solution.d_ss = best_ss;
    out->solution.d_ff = best_ff;
    out->solution.status = out->timed_out ? 2 : 0;
    if (stalled)
        out->solution.solver_name = "phase2_hybrid_sa(stall)";
    else
        out->solution.solver_name =
            out->timed_out ? "phase2_hybrid_sa(timed_out)" : "phase2_hybrid_sa";

    return 0;
}

void sa_solution_free(SaSolveResult *out)
{
    if (!out)
        return;
    out->solution.clear();
}

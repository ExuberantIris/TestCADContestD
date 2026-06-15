#include "setup_lp_solve.hpp"

#include "lp_score.hpp"
#include "pd_output.h"
#include "sa_config.hpp"
#include "sa_eval.hpp"

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

std::vector<int> topo_sort_pseudo_dag(int n_nodes, const std::vector<DgEdge> &path_edges,
                                      const std::vector<std::vector<std::pair<int, double>>> &out)
{
    std::vector<int> indeg(static_cast<std::size_t>(n_nodes), 0);
    for (const DgEdge &e : path_edges)
        indeg[static_cast<std::size_t>(e.to)]++;

    std::vector<char> in_order(static_cast<std::size_t>(n_nodes), 0);
    std::vector<int> order;
    order.reserve(static_cast<std::size_t>(n_nodes));

    std::queue<int> q;
    for (int i = 0; i < n_nodes; i++) {
        if (indeg[static_cast<std::size_t>(i)] == 0)
            q.push(i);
    }

    while (!q.empty()) {
        const int u = q.front();
        q.pop();
        order.push_back(u);
        in_order[static_cast<std::size_t>(u)] = 1;
        for (const auto &nv : out[static_cast<std::size_t>(u)]) {
            const int v = nv.first;
            if (--indeg[static_cast<std::size_t>(v)] == 0)
                q.push(v);
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

bool compute_ideal_ff_arrival(const PdDesign *d, const LpProblem *pb, std::vector<double> *ideal)
{
    const int root = find_root(d);
    const int n_nodes = d->n_nodes;
    const int n_paths = static_cast<int>(pb->path_ids.size());

    std::vector<DgEdge> path_edges;
    std::vector<std::vector<std::pair<int, double>>> out_adj(static_cast<std::size_t>(n_nodes));
    path_edges.reserve(static_cast<std::size_t>(n_paths));

    for (int pi = 0; pi < n_paths; pi++) {
        const PdPath *path = &d->paths[pb->path_ids[static_cast<std::size_t>(pi)]];
        if (path->launch_id < 0 || path->capture_id < 0)
            continue;

        DgEdge e;
        e.from = path->launch_id;
        e.to = path->capture_id;
        e.weight = path_required_skew(d, path);
        path_edges.push_back(e);
        out_adj[static_cast<std::size_t>(e.from)].push_back({e.to, e.weight});
    }

    const std::vector<int> order = topo_sort_pseudo_dag(n_nodes, path_edges, out_adj);
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
                              std::vector<double> *target_d_ss, int node_id)
{
    const PdNode *n = &d->nodes[node_id];

    if (n->kind == PD_NODE_FF)
        return ideal_arrival[static_cast<std::size_t>(node_id)];

    std::vector<double> child_req;
    std::vector<int> child_ids;
    child_req.reserve(static_cast<std::size_t>(n->nchildren));
    child_ids.reserve(static_cast<std::size_t>(n->nchildren));

    for (int i = 0; i < n->nchildren; i++) {
        const int child = n->children[i];
        child_req.push_back(build_seg_tree_targets(d, pb, ideal_arrival, target_d_ss, child));
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
                                std::vector<double> *target_d_ss)
{
    target_d_ss->assign(pb->branches.size(), 0.0);
    build_seg_tree_targets(d, pb, ideal_arrival, target_d_ss, find_root(d));
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

double timing_score(const LpProblem *pb, const SaPgCtx &ctx)
{
    return lp_compute_timing_score(ctx.wns_ss, ctx.tns_ss, ctx.wns_ff, ctx.tns_ff, pb->wns_ss_ori,
                                   pb->tns_ss_ori, pb->wns_ff_ori, pb->tns_ff_ori);
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

int seg_tree_write_outputs(const PdDesign *d, const LpProblem *pb, const char *out_dir, char *err,
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
    if (!compute_ideal_ff_arrival(d, pb, &ideal_arrival)) {
        if (err && err_sz > 0)
            std::snprintf(err, err_sz, "failed to compute ideal FF arrival");
        return -1;
    }

    std::vector<double> seg_d_ss;
    build_all_seg_tree_targets(d, pb, ideal_arrival, &seg_d_ss);

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

    std::vector<double> ideal_arrival;
    if (!compute_ideal_ff_arrival(d, pb, &ideal_arrival)) {
        if (err && err_sz > 0)
            std::snprintf(err, err_sz, "failed to compute ideal FF arrival");
        return -1;
    }

    std::vector<double> target_d_ss;
    build_all_seg_tree_targets(d, pb, ideal_arrival, &target_d_ss);

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

void sa_solution_free(SaSolveResult *out)
{
    if (!out)
        return;
    out->solution.clear();
}

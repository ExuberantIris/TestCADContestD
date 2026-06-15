#include "setup_lp_solve.hpp"

#include "lp_score.hpp"
#include "sa_config.hpp"
#include "sa_eval.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <queue>
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

void propagate_tree_arrivals(const PdDesign *d, const LpProblem *pb, const std::vector<double> &d_ss,
                             std::vector<double> *arrival)
{
    const int root = find_root(d);
    const int n_nodes = d->n_nodes;
    arrival->assign(static_cast<std::size_t>(n_nodes), 0.0);

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
            for (std::size_t b = 0; b < pb->branches.size(); b++) {
                const LpBranch &br = pb->branches[b];
                if (br.parent_node == u && br.child_node == v) {
                    delay = d_ss[b];
                    break;
                }
            }
            (*arrival)[static_cast<std::size_t>(v)] =
                (*arrival)[static_cast<std::size_t>(u)] + delay;
            q.push(v);
        }
    }
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

void tree_delays_from_arrivals(const PdDesign *d, const LpProblem *pb,
                               const std::vector<double> &ff_arrival,
                               const std::vector<double> &cur_ss, std::vector<double> *d_ss)
{
    const int root = find_root(d);
    const int n_nodes = d->n_nodes;
    std::vector<double> tree_t(static_cast<std::size_t>(n_nodes), 0.0);
    tree_t[static_cast<std::size_t>(root)] = 0.0;

    std::queue<int> q;
    q.push(root);

    while (!q.empty()) {
        const int u = q.front();
        q.pop();
        const PdNode *nu = &d->nodes[u];
        for (int i = 0; i < nu->nchildren; i++) {
            const int v = nu->children[i];
            const PdNode *nv = &d->nodes[v];

            for (std::size_t b = 0; b < pb->branches.size(); b++) {
                const LpBranch &br = pb->branches[b];
                if (br.parent_node != u || br.child_node != v)
                    continue;

                double delay = 0.0;
                if (nv->kind == PD_NODE_FF) {
                    delay = ff_arrival[static_cast<std::size_t>(v)] -
                            tree_t[static_cast<std::size_t>(u)];
                } else {
                    delay = std::max(br.d_ss_min, cur_ss[b]);
                }
                delay = std::clamp(delay, br.d_ss_min, br.d_ss_max);
                (*d_ss)[b] = delay;
                tree_t[static_cast<std::size_t>(v)] = tree_t[static_cast<std::size_t>(u)] + delay;
                break;
            }
            q.push(v);
        }
    }
}

double setup_objective(const LpProblem *pb, const PdDesign *d, const SaPgCtx &ctx,
                       const LpBufferChainDp *dp_ss, const LpBufferChainDp *dp_ff,
                       const std::vector<double> &d_ss, const std::vector<double> &d_ff)
{
    SaPgCtx trial = ctx;
    sa_eval_state(pb, d, d_ss, d_ff, dp_ss, dp_ff, &trial);
    return trial.tns_ss + 0.01 * trial.wns_ss;
}

} // namespace

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

    const int root = find_root(d);
    const int n_nodes = d->n_nodes;
    const int n_paths = static_cast<int>(pb->path_ids.size());

    std::vector<double> cur_ss = initial->d_ss;
    std::vector<double> cur_ff = initial->d_ff;
    std::vector<double> best_ss = cur_ss;
    std::vector<double> best_ff = cur_ff;

    double best_obj = setup_objective(pb, d, ctx, dp_ss, dp_ff, cur_ss, cur_ff);

    std::vector<DgEdge> path_edges;
    std::vector<std::vector<std::pair<int, double>>> out_adj;
    std::vector<double> ff_arrival;
    std::vector<double> actual_arrival;

    const Clock::time_point t0 = Clock::now();

    while (elapsed_sec(t0) < time_limit_sec) {
        out->iterations++;

        propagate_tree_arrivals(d, pb, cur_ss, &actual_arrival);

        path_edges.clear();
        out_adj.assign(static_cast<std::size_t>(n_nodes), {});

        for (int pi = 0; pi < n_paths; pi++) {
            const PdPath *path = &d->paths[pb->path_ids[static_cast<std::size_t>(pi)]];
            if (path->launch_id < 0 || path->capture_id < 0)
                continue;

            double q = path_required_skew(d, path);
            const double skew =
                actual_arrival[static_cast<std::size_t>(path->capture_id)] -
                actual_arrival[static_cast<std::size_t>(path->launch_id)];
            if (skew + 1e-9 < q)
                q = q + (q - skew);

            DgEdge e;
            e.from = path->launch_id;
            e.to = path->capture_id;
            e.weight = q;
            path_edges.push_back(e);
            out_adj[static_cast<std::size_t>(e.from)].push_back({e.to, e.weight});
        }

        const std::vector<int> order = topo_sort_pseudo_dag(n_nodes, path_edges, out_adj);
        relax_path_arrivals_topo(order, out_adj, root, n_nodes, &ff_arrival);
        tree_delays_from_arrivals(d, pb, ff_arrival, cur_ss, &cur_ss);

        const double obj = setup_objective(pb, d, ctx, dp_ss, dp_ff, cur_ss, cur_ff);
        if (obj > best_obj + 1e-12) {
            best_obj = obj;
            best_ss = cur_ss;
            best_ff = cur_ff;
        }
    }

    out->elapsed_sec = elapsed_sec(t0);
    out->timed_out = (out->elapsed_sec >= time_limit_sec - 1e-6) ? 1 : 0;
    out->solution.d_ss = best_ss;
    out->solution.d_ff = best_ff;
    out->solution.status = out->timed_out ? 2 : 0;
    out->solution.solver_name =
        out->timed_out ? "setup_topo_path(timed_out)" : "setup_topo_path";

    return 0;
}

void sa_solution_free(SaSolveResult *out)
{
    if (!out)
        return;
    out->solution.clear();
}

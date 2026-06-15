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

bool is_leaf_ff(const PdDesign *d, int node_id)
{
    const PdNode *n = &d->nodes[node_id];
    return n->kind == PD_NODE_FF && n->nchildren == 0;
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

void branches_to_node(const PdDesign *d, const LpProblem *pb, int node_id, std::vector<int> *branches)
{
    branches->clear();
    int cur = node_id;
    int steps = 0;
    while (cur >= 0 && steps++ < d->n_nodes) {
        const int parent = d->nodes[cur].parent;
        if (parent < 0)
            break;
        for (std::size_t b = 0; b < pb->branches.size(); b++) {
            const LpBranch &br = pb->branches[b];
            if (br.parent_node == parent && br.child_node == cur &&
                br.kind == LpBranchKind::ExistingBuf) {
                branches->push_back(static_cast<int>(b));
                break;
            }
        }
        cur = parent;
    }
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

double timing_score(const LpProblem *pb, const SaPgCtx &ctx)
{
    return lp_compute_timing_score(ctx.wns_ss, ctx.tns_ss, ctx.wns_ff, ctx.tns_ff, pb->wns_ss_ori,
                                   pb->tns_ss_ori, pb->wns_ff_ori, pb->tns_ff_ori);
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

    std::vector<BranchDpOpts> opts;
    sa_build_branch_opts(pb, d, &opts);

    std::vector<double> ideal_arrival;
    if (!compute_ideal_ff_arrival(d, pb, &ideal_arrival)) {
        if (err && err_sz > 0)
            std::snprintf(err, err_sz, "failed to compute ideal FF arrival");
        return -1;
    }

    std::vector<double> cur_ss = initial->d_ss;
    std::vector<double> cur_ff = initial->d_ff;
    std::vector<double> best_ss = cur_ss;
    std::vector<double> best_ff = cur_ff;

    sa_eval_state(pb, d, cur_ss, cur_ff, dp_ss, dp_ff, &ctx);
    double best_score = timing_score(pb, ctx);

    const int n_br = static_cast<int>(pb->branches.size());
    std::vector<int> branch_stall(static_cast<std::size_t>(n_br), 0);
    std::vector<double> current_arrival;
    std::vector<int> leaf_ffs;
    leaf_ffs.reserve(static_cast<std::size_t>(pb->ff_node_ids.size()));
    for (int ff_node : pb->ff_node_ids) {
        if (is_leaf_ff(d, ff_node))
            leaf_ffs.push_back(ff_node);
    }

    std::mt19937 rng(static_cast<unsigned>(
        std::chrono::steady_clock::now().time_since_epoch().count()));
    std::uniform_real_distribution<double> uni01(0.0, 1.0);

    const Clock::time_point t0 = Clock::now();
    double temperature = SaConfig::kSaTemperatureInit;
    int no_improve_iters = 0;
    int stalled = 0;

    while (elapsed_sec(t0) < time_limit_sec) {
        out->iterations++;

        propagate_tree_arrivals(d, pb, cur_ss, &current_arrival);

        std::vector<std::pair<double, int>> ranked;
        ranked.reserve(leaf_ffs.size());
        for (int ff_node : leaf_ffs) {
            const double gap = ideal_arrival[static_cast<std::size_t>(ff_node)] -
                               current_arrival[static_cast<std::size_t>(ff_node)];
            ranked.push_back({std::fabs(gap), ff_node});
        }
        std::sort(ranked.begin(), ranked.end(),
                  [](const std::pair<double, int> &a, const std::pair<double, int> &b) {
                      if (a.first != b.first)
                          return a.first > b.first;
                      return a.second < b.second;
                  });

        const int pick_k =
            std::min(SaConfig::kSaLeafFfPickCount, static_cast<int>(ranked.size()));
        std::vector<char> branch_eligible(static_cast<std::size_t>(n_br), 0);
        std::vector<int> target_ffs;
        target_ffs.reserve(static_cast<std::size_t>(pick_k));

        std::vector<int> path_br;
        for (int i = 0; i < pick_k; i++) {
            const int ff_node = ranked[static_cast<std::size_t>(i)].second;
            target_ffs.push_back(ff_node);
            branches_to_node(d, pb, ff_node, &path_br);
            for (int b : path_br) {
                if (branch_stall[static_cast<std::size_t>(b)] < SaConfig::kSaBranchNoImproveLimit)
                    branch_eligible[static_cast<std::size_t>(b)] = 1;
            }
        }

        std::vector<int> eligible_br;
        for (int b = 0; b < n_br; b++) {
            if (branch_eligible[static_cast<std::size_t>(b)])
                eligible_br.push_back(b);
        }
        if (eligible_br.empty())
            break;

        const int moved_branch = eligible_br[static_cast<std::size_t>(
            std::uniform_int_distribution<int>(0, static_cast<int>(eligible_br.size()) - 1)(rng))];

        int target_ff = target_ffs.front();
        for (int ff_node : target_ffs) {
            branches_to_node(d, pb, ff_node, &path_br);
            for (int b : path_br) {
                if (b == moved_branch) {
                    target_ff = ff_node;
                    break;
                }
            }
        }

        const double gap = ideal_arrival[static_cast<std::size_t>(target_ff)] -
                           current_arrival[static_cast<std::size_t>(target_ff)];
        const auto &o = opts[static_cast<std::size_t>(moved_branch)];

        std::vector<double> trial_ss = cur_ss;
        bool moved = false;
        if (gap > 1e-9 && o.ss_delays.size() > 1) {
            trial_ss[static_cast<std::size_t>(moved_branch)] =
                next_higher(trial_ss[static_cast<std::size_t>(moved_branch)], o.ss_delays);
            moved = true;
        } else if (gap < -1e-9 && o.ss_delays.size() > 1) {
            trial_ss[static_cast<std::size_t>(moved_branch)] =
                next_lower(trial_ss[static_cast<std::size_t>(moved_branch)], o.ss_delays);
            moved = true;
        }
        if (!moved)
            continue;

        const double old_score = timing_score(pb, ctx);
        SaPgCtx trial_ctx = ctx;
        sa_eval_state(pb, d, trial_ss, cur_ff, dp_ss, dp_ff, &trial_ctx);
        const double new_score = timing_score(pb, trial_ctx);
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
            if (no_improve_iters >= SaConfig::kSaNoImproveLimit) {
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
        out->solution.solver_name = "setup_topo_ff_sa(stall)";
    else
        out->solution.solver_name =
            out->timed_out ? "setup_topo_ff_sa(timed_out)" : "setup_topo_ff_sa";

    return 0;
}

void sa_solution_free(SaSolveResult *out)
{
    if (!out)
        return;
    out->solution.clear();
}

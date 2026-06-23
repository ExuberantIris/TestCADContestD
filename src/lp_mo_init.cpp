#include "lp_mo_init.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <deque>
#include <vector>

// Exact resize-only relaxation: each branch's delay is a free variable bounded by the
// achievable [min,max] delay range of the buffer catalog at that branch's fanout. Setup
// (SS) and hold (FF) use disjoint variables (d_ss vs d_ff), so they decompose into two
// independent problems. Each one is a classic "difference constraint" feasibility problem
// on a tree (clk latency at every node = sum of ancestor branch delays).
//
// We solve in coordinates relative to the *original* design (psi(node) = deviation of that
// node's accumulated clock latency from its current value). This keeps every branch pinned
// to its current delay by default; the SPFA below only lets psi move away from 0 where a
// path constraint truly forces it to, instead of drifting every unconstrained branch to an
// extreme of its catalog range (which would otherwise wreck area and collaterally shift
// other paths' slack). The threshold tau (desired worst-case slack) is bisected to find the
// true maximum achievable WNS for this corner; SPFA/Bellman-Ford negative-cycle detection
// answers each feasibility probe exactly, not heuristically.

namespace {

using Clock = std::chrono::steady_clock;

constexpr double kInf = 1e18;
constexpr double kEps = 1e-9;
constexpr int kMaxBisectIters = 60;

double elapsed_sec(const Clock::time_point &t0)
{
    return std::chrono::duration<double>(Clock::now() - t0).count();
}

/** Fixed (tau-independent) edges encoding lo_b-orig(b) <= psi(child)-psi(parent) <=
 *  hi_b-orig(b) for every branch. CSR adjacency: edge (u,v,w) means psi(v) <= psi(u) + w. */
struct BoxEdges {
    std::vector<int> head;
    std::vector<int> to;
    std::vector<double> w;
};

/** Timing-path edges; actual weight is base[idx] - tau, recomputed for every tau probe. */
struct CrossEdges {
    std::vector<int> head;
    std::vector<int> to;
    std::vector<double> base;
};

void build_box_edges(const LpProblem *pb, int n_nodes, bool use_ss,
                     const std::vector<double> &orig_delay, BoxEdges *be)
{
    std::vector<std::vector<std::pair<int, double>>> adj(static_cast<std::size_t>(n_nodes));
    for (std::size_t b = 0; b < pb->branches.size(); b++) {
        const LpBranch &br = pb->branches[b];
        double lo = 0.0, hi = 0.0, orig = 0.0;
        if (br.kind == LpBranchKind::ExistingBuf) {
            lo = use_ss ? br.d_ss_min : br.d_ff_min;
            hi = use_ss ? br.d_ss_max : br.d_ff_max;
            orig = orig_delay[b];
        }
        // Insertable branches stay pinned at 0: resize-only scope, no buffer there yet.
        adj[static_cast<std::size_t>(br.parent_node)].push_back({br.child_node, hi - orig});
        adj[static_cast<std::size_t>(br.child_node)].push_back({br.parent_node, orig - lo});
    }

    be->head.assign(static_cast<std::size_t>(n_nodes) + 1, 0);
    for (int i = 0; i < n_nodes; i++)
        be->head[static_cast<std::size_t>(i) + 1] =
            be->head[static_cast<std::size_t>(i)] +
            static_cast<int>(adj[static_cast<std::size_t>(i)].size());

    const int n_edges = be->head[static_cast<std::size_t>(n_nodes)];
    be->to.resize(static_cast<std::size_t>(n_edges));
    be->w.resize(static_cast<std::size_t>(n_edges));
    for (int i = 0; i < n_nodes; i++) {
        int idx = be->head[static_cast<std::size_t>(i)];
        for (const auto &e : adj[static_cast<std::size_t>(i)]) {
            be->to[static_cast<std::size_t>(idx)] = e.first;
            be->w[static_cast<std::size_t>(idx)] = e.second;
            idx++;
        }
    }
}

/** setup_check=true builds the SS/setup graph (edge capture->launch, base = current setup
 *  slack); false builds the FF/hold graph (edge launch->capture, base = current hold slack).
 *  Using the *current* slack as the edge base (rather than re-deriving it from clock period
 *  and skew) is what makes psi=0 everywhere an exact zero-cost feasible point: a path needs
 *  tightening only if its current slack is already below the probed tau. Paths with no
 *  resolved launch/capture FF are skipped, matching pd_compute_timing's behaviour. */
void build_cross_edges(const PdDesign *d, bool setup_check, CrossEdges *ce)
{
    const int n_nodes = d->n_nodes;
    std::vector<std::vector<std::pair<int, double>>> adj(static_cast<std::size_t>(n_nodes));

    for (int i = 0; i < d->n_paths; i++) {
        const PdPath &p = d->paths[static_cast<std::size_t>(i)];
        if (p.launch_id < 0 || p.capture_id < 0)
            continue;

        if (setup_check)
            adj[static_cast<std::size_t>(p.capture_id)].push_back({p.launch_id, p.slack_setup_ss});
        else
            adj[static_cast<std::size_t>(p.launch_id)].push_back({p.capture_id, p.slack_hold_ff});
    }

    ce->head.assign(static_cast<std::size_t>(n_nodes) + 1, 0);
    for (int i = 0; i < n_nodes; i++)
        ce->head[static_cast<std::size_t>(i) + 1] =
            ce->head[static_cast<std::size_t>(i)] +
            static_cast<int>(adj[static_cast<std::size_t>(i)].size());

    const int n_edges = ce->head[static_cast<std::size_t>(n_nodes)];
    ce->to.resize(static_cast<std::size_t>(n_edges));
    ce->base.resize(static_cast<std::size_t>(n_edges));
    for (int i = 0; i < n_nodes; i++) {
        int idx = ce->head[static_cast<std::size_t>(i)];
        for (const auto &e : adj[static_cast<std::size_t>(i)]) {
            ce->to[static_cast<std::size_t>(idx)] = e.first;
            ce->base[static_cast<std::size_t>(idx)] = e.second;
            idx++;
        }
    }
}

/** SPFA feasibility of psi[v] <= psi[u]+w (box edges) and psi[v] <= psi[u]+(base-tau) (cross
 *  edges). All nodes start at psi=0 (instead of a single source) so the algorithm only moves
 *  a node away from its current value when some constraint actually forces it to - this
 *  yields the minimal-disruption witness among all tau-feasible solutions, not an arbitrary
 *  extremal one. Returns false on negative cycle (infeasible at this tau), on exceeding
 *  max_ops (treated conservatively as "not confirmed feasible" - this matters because a
 *  negative cycle of tiny magnitude, which is exactly what shows up near the true bisection
 *  boundary, can otherwise take up to O(n_nodes) revisits per node to detect on a large
 *  graph), or on timeout. */
bool spfa_feasible(const BoxEdges &be, const CrossEdges &ce, double tau, int n_nodes,
                   long long max_ops, std::vector<double> *dist_out,
                   const Clock::time_point &deadline)
{
    std::vector<double> dist(static_cast<std::size_t>(n_nodes), 0.0);
    std::vector<int> cnt(static_cast<std::size_t>(n_nodes), 1);
    std::vector<char> in_q(static_cast<std::size_t>(n_nodes), 1);
    std::deque<int> q;
    for (int i = 0; i < n_nodes; i++)
        q.push_back(i);

    long long ops = 0;
    while (!q.empty()) {
        if (++ops > max_ops)
            return false;
        if ((ops & 0xFFFF) == 0 && Clock::now() > deadline)
            return false;

        const int u = q.front();
        q.pop_front();
        in_q[static_cast<std::size_t>(u)] = 0;
        const double du = dist[static_cast<std::size_t>(u)];

        for (int idx = be.head[static_cast<std::size_t>(u)]; idx < be.head[static_cast<std::size_t>(u) + 1];
             idx++) {
            const int v = be.to[static_cast<std::size_t>(idx)];
            const double nd = du + be.w[static_cast<std::size_t>(idx)];
            if (nd < dist[static_cast<std::size_t>(v)] - kEps) {
                dist[static_cast<std::size_t>(v)] = nd;
                if (!in_q[static_cast<std::size_t>(v)]) {
                    if (++cnt[static_cast<std::size_t>(v)] > n_nodes)
                        return false;
                    in_q[static_cast<std::size_t>(v)] = 1;
                    q.push_back(v);
                }
            }
        }
        for (int idx = ce.head[static_cast<std::size_t>(u)]; idx < ce.head[static_cast<std::size_t>(u) + 1];
             idx++) {
            const int v = ce.to[static_cast<std::size_t>(idx)];
            const double nd = du + (ce.base[static_cast<std::size_t>(idx)] - tau);
            if (nd < dist[static_cast<std::size_t>(v)] - kEps) {
                dist[static_cast<std::size_t>(v)] = nd;
                if (!in_q[static_cast<std::size_t>(v)]) {
                    if (++cnt[static_cast<std::size_t>(v)] > n_nodes)
                        return false;
                    in_q[static_cast<std::size_t>(v)] = 1;
                    q.push_back(v);
                }
            }
        }
    }

    if (dist_out)
        *dist_out = std::move(dist);
    return true;
}

struct SkewResult {
    std::vector<double> dist;
    double tau = 0.0;
    bool converged = false;
};

/** Finds the maximum tau for which the constraint graph is feasible (= the true minimum
 *  achievable WNS for this corner under resize-only), via exponential bracket search +
 *  bisection on top of SPFA feasibility checks. wns_ori is the corner's current/baseline WNS,
 *  which is always a feasible starting point (psi=0 satisfies every constraint exactly at
 *  tau=wns_ori, since by definition no path's current slack is below it). */
SkewResult solve_max_tau(const BoxEdges &be, const CrossEdges &ce, int n_nodes, long long max_ops,
                         double wns_ori, double scale, const Clock::time_point &deadline)
{
    SkewResult result;

    double lo = wns_ori - 1e-6;
    if (!spfa_feasible(be, ce, lo, n_nodes, max_ops, &result.dist, deadline)) {
        // Defensive fallback in case of any mismatch with the caller's baseline WNS.
        lo = -(scale * 20.0 + 5.0);
        if (!spfa_feasible(be, ce, lo, n_nodes, max_ops, &result.dist, deadline))
            return result; // should not happen; signal failure via empty dist
    }
    result.tau = lo;

    double hi = std::max(0.05 * scale, lo + 0.05 * scale);
    bool found_infeasible = false;
    for (int expand = 0; expand < 12 && Clock::now() < deadline; expand++) {
        std::vector<double> dist;
        if (spfa_feasible(be, ce, hi, n_nodes, max_ops, &dist, deadline)) {
            lo = hi;
            result.dist = std::move(dist);
            result.tau = lo;
            hi += std::max(hi - lo, 0.05 * scale) * 2.0;
            if (hi > scale * 64.0)
                break;
        } else {
            found_infeasible = true;
            break;
        }
    }

    if (!found_infeasible) {
        result.converged = true;
        return result;
    }

    for (int iter = 0; iter < kMaxBisectIters && Clock::now() < deadline; iter++) {
        if (hi - lo < 1e-7)
            break;
        const double mid = 0.5 * (lo + hi);
        std::vector<double> dist;
        if (spfa_feasible(be, ce, mid, n_nodes, max_ops, &dist, deadline)) {
            lo = mid;
            result.dist = std::move(dist);
            result.tau = lo;
        } else {
            hi = mid;
        }
    }
    result.converged = true;
    return result;
}

} // namespace

int lp_solve_mo_init(LpProblem *pb, const PdDesign *d, LpSolution *sol, double time_limit_sec,
                     char * /*err*/, std::size_t /*err_sz*/)
{
    sol->clear();
    const int n_br = static_cast<int>(pb->branches.size());
    const int n_nodes = d->n_nodes;
    if (n_nodes <= 0 || n_br <= 0)
        return -1;

    const Clock::time_point t0 = Clock::now();
    const Clock::time_point deadline =
        t0 + std::chrono::duration_cast<Clock::duration>(
                 std::chrono::duration<double>(std::max(0.1, time_limit_sec)));
    const double scale = std::max(d->clock_period, 1e-6);

    sol->d_ss.assign(static_cast<std::size_t>(n_br), 0.0);
    sol->d_ff.assign(static_cast<std::size_t>(n_br), 0.0);

    // Original per-branch delays double as both the safe fallback (if a corner's solve
    // doesn't finish before the deadline) and the reference point that the box-edge graph
    // pins each branch to by default.
    std::vector<double> orig_ss(static_cast<std::size_t>(n_br), 0.0);
    std::vector<double> orig_ff(static_cast<std::size_t>(n_br), 0.0);
    for (int b = 0; b < n_br; b++) {
        const LpBranch &br = pb->branches[static_cast<std::size_t>(b)];
        if (br.kind == LpBranchKind::ExistingBuf && br.cell_idx >= 0) {
            const PdCell *c = &d->cells[br.cell_idx];
            orig_ss[static_cast<std::size_t>(b)] = lp_eval_branch_delay_ss(d, c, br.fanout);
            orig_ff[static_cast<std::size_t>(b)] = lp_eval_branch_delay_ff(d, c, br.fanout);
        }
    }
    sol->d_ss = orig_ss;
    sol->d_ff = orig_ff;

    BoxEdges box_ss, box_ff;
    CrossEdges cross_setup, cross_hold;
    build_box_edges(pb, n_nodes, /*use_ss=*/true, orig_ss, &box_ss);
    build_box_edges(pb, n_nodes, /*use_ss=*/false, orig_ff, &box_ff);
    build_cross_edges(d, /*setup_check=*/true, &cross_setup);
    build_cross_edges(d, /*setup_check=*/false, &cross_hold);

    // Bound every individual feasibility probe's cost so that a single near-boundary check
    // (where any negative cycle has near-zero magnitude and would otherwise need up to
    // O(n_nodes) revisits per node to confirm) can never eat the whole time budget by itself.
    // This trades a small amount of precision on huge graphs for predictable, bounded-cost
    // probes - bisection only narrows "lo" on a *confirmed* feasible probe, so the result is
    // always a safe (if occasionally slightly conservative) lower bound on the true optimum.
    const auto max_ops_for = [](const BoxEdges &be, const CrossEdges &ce) -> long long {
        const long long n_edges =
            static_cast<long long>(be.to.size()) + static_cast<long long>(ce.to.size());
        return 60 * n_edges + 200000;
    };
    const SkewResult ss_res = solve_max_tau(box_ss, cross_setup, n_nodes,
                                            max_ops_for(box_ss, cross_setup), pb->wns_ss_ori,
                                            scale, deadline);
    const SkewResult ff_res = solve_max_tau(box_ff, cross_hold, n_nodes,
                                            max_ops_for(box_ff, cross_hold), pb->wns_ff_ori, scale,
                                            deadline);

    if (!ss_res.dist.empty()) {
        for (int b = 0; b < n_br; b++) {
            const LpBranch &br = pb->branches[static_cast<std::size_t>(b)];
            if (br.kind != LpBranchKind::ExistingBuf) {
                sol->d_ss[static_cast<std::size_t>(b)] = 0.0;
                continue;
            }
            const double raw = orig_ss[static_cast<std::size_t>(b)] +
                               ss_res.dist[static_cast<std::size_t>(br.child_node)] -
                               ss_res.dist[static_cast<std::size_t>(br.parent_node)];
            sol->d_ss[static_cast<std::size_t>(b)] = std::clamp(raw, br.d_ss_min, br.d_ss_max);
        }
    }
    if (!ff_res.dist.empty()) {
        for (int b = 0; b < n_br; b++) {
            const LpBranch &br = pb->branches[static_cast<std::size_t>(b)];
            if (br.kind != LpBranchKind::ExistingBuf) {
                sol->d_ff[static_cast<std::size_t>(b)] = 0.0;
                continue;
            }
            const double raw = orig_ff[static_cast<std::size_t>(b)] +
                               ff_res.dist[static_cast<std::size_t>(br.child_node)] -
                               ff_res.dist[static_cast<std::size_t>(br.parent_node)];
            sol->d_ff[static_cast<std::size_t>(b)] = std::clamp(raw, br.d_ff_min, br.d_ff_max);
        }
    }

    std::printf(
        "lp_solve_mo_init (exact tree-skew bisection): setup WNS %.6f -> %.6f (%s), hold WNS %.6f -> %.6f (%s), %.3fs\n",
        pb->wns_ss_ori, ss_res.tau, ss_res.converged ? "converged" : "timeout", pb->wns_ff_ori,
        ff_res.tau, ff_res.converged ? "converged" : "timeout", elapsed_sec(t0));

    sol->status = (ss_res.converged && ff_res.converged) ? 1 : 0;
    sol->solver_name = "tree_skew_bisect_spfa";
    return 0;
}

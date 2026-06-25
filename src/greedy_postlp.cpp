#include "greedy_postlp.hpp"

#include "lp_score.hpp"
#include "pd_clock.h"
#include "pd_timing.h"
#include "pd_util.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <random>
#include <set>
#include <unordered_map>
#include <vector>

using Clock = std::chrono::steady_clock;

static double elapsed_sec(const Clock::time_point &t0)
{
    return std::chrono::duration<double>(Clock::now() - t0).count();
}

static bool before_deadline(const Clock::time_point &deadline)
{
    return Clock::now() < deadline;
}

static void collect_subtree_nodes(const PdDesign *d, int node_id, std::vector<int> *nodes)
{
    const PdNode *n = &d->nodes[node_id];
    nodes->push_back(node_id);
    for (int i = 0; i < n->nchildren; i++)
        collect_subtree_nodes(d, n->children[i], nodes);
}

static void refresh_design_timing(PdDesign *d)
{
    pd_compute_timing(d);
}

/** A path whose slack changes when a given branch's delay changes by some delta. Resizing a
 *  buffer shifts the accumulated clock latency of its *entire* subtree by the same amount, so
 *  a path is only affected through whichever endpoint (launch/capture) lies in that subtree;
 *  sign is +1 if the capture FF is downstream, -1 if the launch FF is downstream. If *both*
 *  endpoints are downstream the shift cancels exactly, so such paths are dropped (sign 0). */
struct AffectedPath {
    int path_idx;
    int sign;
};

static double multiset_min_or_zero(const std::multiset<double> &s)
{
    return s.empty() ? 0.0 : *s.begin();
}

/** Applies delta_ss/delta_ff to every path in `affected` (per AffectedPath::sign) and keeps
 *  the running multisets/TNS totals in sync. Calling this again with the negated deltas
 *  exactly undoes it, which is how candidate evaluation stays O(|affected|) instead of
 *  O(n_paths): try a delta, read the score, then undo it - all without ever re-deriving
 *  anything from d_clk_ss/d_clk_ff or re-walking the subtree. */
static void apply_affected_delta(PdDesign *d, const std::vector<AffectedPath> &affected,
                                 double delta_ss, double delta_ff,
                                 std::multiset<double> *setup_slacks,
                                 std::multiset<double> *hold_slacks, double *tns_setup_total,
                                 double *tns_hold_total)
{
    for (const AffectedPath &ap : affected) {
        PdPath &p = d->paths[static_cast<std::size_t>(ap.path_idx)];

        const double old_setup = p.slack_setup_ss;
        const double new_setup = old_setup + ap.sign * delta_ss;
        setup_slacks->erase(setup_slacks->find(old_setup));
        if (old_setup < 0.0) *tns_setup_total -= old_setup;
        setup_slacks->insert(new_setup);
        if (new_setup < 0.0) *tns_setup_total += new_setup;
        p.slack_setup_ss = new_setup;

        const double old_hold = p.slack_hold_ff;
        const double new_hold = old_hold - ap.sign * delta_ff;
        hold_slacks->erase(hold_slacks->find(old_hold));
        if (old_hold < 0.0) *tns_hold_total -= old_hold;
        hold_slacks->insert(new_hold);
        if (new_hold < 0.0) *tns_hold_total += new_hold;
        p.slack_hold_ff = new_hold;
    }
}

// 🚀 優化二：拿掉耗時的字串拷貝，專心做數值更新
static void apply_cell_to_branch(PdDesign *d, const LpBranch &br, int cell_idx,
                                 std::vector<double> *cur_ss, std::vector<double> *cur_ff, int b)
{
    PdNode *node = &d->nodes[br.child_node];
    if (node->kind != PD_NODE_BUF)
        return;

    node->cell_idx = cell_idx;
    (*cur_ss)[static_cast<std::size_t>(b)] =
        lp_eval_branch_delay_ss(d, &d->cells[cell_idx], br.fanout);
    (*cur_ff)[static_cast<std::size_t>(b)] =
        lp_eval_branch_delay_ff(d, &d->cells[cell_idx], br.fanout);
}

static double score_design(const LpProblem *pb, const PdDesign *d)
{
    return lp_compute_score(d->wns_setup_ss, d->tns_setup_ss, d->wns_hold_ff, d->tns_hold_ff,
                            d->total_area, pb->wns_ss_ori, pb->tns_ss_ori, pb->wns_ff_ori,
                            pb->tns_ff_ori, pb->area_ori);
}

static void build_branch_candidates(const PdDesign *d, const LpProblem *pb,
                                    std::vector<std::vector<int>> *candidates)
{
    const int n_br = static_cast<int>(pb->branches.size());
    candidates->assign(static_cast<std::size_t>(n_br), {});

    for (int b = 0; b < n_br; b++) {
        const LpBranch &br = pb->branches[static_cast<std::size_t>(b)];
        if (br.kind != LpBranchKind::ExistingBuf)
            continue;

        std::unordered_map<long long, int> best_cell;
        for (int ci = 0; ci < d->n_cells; ci++) {
            const PdCell *c = &d->cells[ci];
            if (!br.allow_zero_cell && pd_cell_is_zero(c))
                continue;
            if (br.fanout > c->max_fanout)
                continue;

            const double ss = lp_eval_branch_delay_ss(d, c, br.fanout);
            const double ff = lp_eval_branch_delay_ff(d, c, br.fanout);
            if (ss < br.d_ss_min - 1e-9 || ss > br.d_ss_max + 1e-9)
                continue;
            if (ff < br.d_ff_min - 1e-9 || ff > br.d_ff_max + 1e-9)
                continue;

            const long long key =
                (static_cast<long long>(std::llround(ss * 1e9)) << 32) ^
                static_cast<long long>(std::llround(ff * 1e9));
            const double area = c->width * c->height;
            const auto it = best_cell.find(key);
            if (it == best_cell.end() || area < d->cells[it->second].width * d->cells[it->second].height)
                best_cell[key] = ci;
        }

        std::vector<int> &out = (*candidates)[static_cast<std::size_t>(b)];
        out.reserve(best_cell.size());
        for (const auto &kv : best_cell)
            out.push_back(kv.second);
        std::sort(out.begin(), out.end(), [d](int a, int bci) {
            const PdCell *ca = &d->cells[a];
            const PdCell *cb = &d->cells[bci];
            const double aa = ca->width * ca->height;
            const double ab = cb->width * cb->height;
            if (aa != ab)
                return aa < ab;
            return a < bci;
        });
    }
}

static void sync_graph_from_delays(PdDesign *d, const LpProblem *pb, std::vector<double> *cur_ss,
                                   std::vector<double> *cur_ff)
{
    const int n_br = static_cast<int>(pb->branches.size());
    for (int b = 0; b < n_br; b++) {
        const LpBranch &br = pb->branches[static_cast<std::size_t>(b)];
        if (br.kind != LpBranchKind::ExistingBuf)
            continue;

        const double target_ss = (*cur_ss)[static_cast<std::size_t>(b)];
        int best_ci = -1;
        double min_err = 1e9;

        for (int ci = 0; ci < d->n_cells; ci++) {
            const PdCell *c = &d->cells[ci];
            if (!br.allow_zero_cell && pd_cell_is_zero(c))
                continue;
            if (br.fanout > c->max_fanout)
                continue;
            const double css = lp_eval_branch_delay_ss(d, c, br.fanout);
            const double err = std::fabs(css - target_ss);
            if (err < min_err) {
                min_err = err;
                best_ci = ci;
            }
        }

        if (best_ci >= 0) {
            apply_cell_to_branch(d, br, best_ci, cur_ss, cur_ff, b);
            std::strncpy(d->nodes[br.child_node].cell, d->cells[best_ci].name, PD_MAX_NAME - 1);
            d->nodes[br.child_node].cell[PD_MAX_NAME - 1] = '\0';
        }
    }
}

int greedy_post_lp(const char *result_dir, const char *testcase_dir, const LpProblem *pb,
                   const PdDesign *d_const, const LpBufferChainDp * /*dp_ss*/,
                   const LpBufferChainDp * /*dp_ff*/, LpSolution *lp_init,
                   const LpMetrics *lp_init_metrics, double time_limit_sec,
                   const Clock::time_point wall_deadline, char *err, std::size_t err_sz)
{
    std::printf("greedy_post_lp: entry (V2: High Performance Cache + Zero String Copy)\n");

    if (!pb || !d_const || !lp_init || !lp_init_metrics || !result_dir) {
        if (err && err_sz > 0)
            std::snprintf(err, err_sz, "null arg");
        return -1;
    }

    PdDesign *d = const_cast<PdDesign *>(d_const);
    const int n_br = static_cast<int>(pb->branches.size());
    std::vector<double> cur_ss = lp_init->d_ss;
    std::vector<double> cur_ff = lp_init->d_ff;

    if (cur_ss.empty() || cur_ff.empty())
        return -1;

    if (!before_deadline(wall_deadline)) {
        if (err && err_sz > 0)
            std::snprintf(err, err_sz, "no time left before greedy");
        return -1;
    }

    std::printf("greedy_post_lp: syncing graph to LP init...\n");
    sync_graph_from_delays(d, pb, &cur_ss, &cur_ff);

    pd_annotate_clock(d);
    refresh_design_timing(d);
    double cur_score = score_design(pb, d);

    if (cur_score < 0.0) {
        std::printf("greedy_post_lp: LP init score worse than baseline; reverting\n");
        for (int b = 0; b < n_br; b++) {
            const LpBranch &br = pb->branches[static_cast<std::size_t>(b)];
            if (br.kind != LpBranchKind::ExistingBuf || br.cell_idx < 0)
                continue;
            apply_cell_to_branch(d, br, br.cell_idx, &cur_ss, &cur_ff, b);
            std::strncpy(d->nodes[br.child_node].cell, d->cells[br.cell_idx].name, PD_MAX_NAME - 1);
            d->nodes[br.child_node].cell[PD_MAX_NAME - 1] = '\0';
        }
        pd_annotate_clock(d);
        refresh_design_timing(d);
        cur_score = score_design(pb, d);
        const_cast<LpMetrics *>(lp_init_metrics)->wns_hold_ff = d->wns_hold_ff;
        const_cast<LpMetrics *>(lp_init_metrics)->tns_hold_ff = d->tns_hold_ff;
    }

    std::vector<std::vector<int>> branch_candidates;
    build_branch_candidates(d, pb, &branch_candidates);

    // 🚀 優化一：在迴圈外預先算好「每個 Branch 會影響到哪幾條 Path」
    // Reverse index FF node -> paths touching it as launch/capture, built once in O(n_paths).
    // Without it, this loop allocated an O(n_nodes) bitmap and rescanned all n_paths paths for
    // *every* branch (O(n_branches*(n_nodes+n_paths))), which dwarfs everything else on large
    // testcases (tens of thousands of branches x hundreds of thousands of paths).
    std::vector<std::vector<int>> launch_paths(d->n_nodes);
    std::vector<std::vector<int>> capture_paths(d->n_nodes);
    for (int p = 0; p < d->n_paths; p++) {
        const PdPath &path = d->paths[p];
        if (path.launch_id >= 0)
            launch_paths[static_cast<std::size_t>(path.launch_id)].push_back(p);
        if (path.capture_id >= 0)
            capture_paths[static_cast<std::size_t>(path.capture_id)].push_back(p);
    }

    std::vector<std::vector<AffectedPath>> branch_affected_paths(n_br);
    {
        std::vector<int> path_stamp(static_cast<std::size_t>(d->n_paths), -1);
        std::vector<int> path_sign(static_cast<std::size_t>(d->n_paths), 0);
        std::vector<int> touched;
        for (int b = 0; b < n_br; b++) {
            const LpBranch &br = pb->branches[static_cast<std::size_t>(b)];
            if (br.kind != LpBranchKind::ExistingBuf) continue;

            std::vector<int> subtree;
            collect_subtree_nodes(d, br.child_node, &subtree);

            touched.clear();
            for (int nid : subtree) {
                if (d->nodes[nid].kind != PD_NODE_FF) continue;
                for (int p : launch_paths[static_cast<std::size_t>(nid)]) {
                    if (path_stamp[static_cast<std::size_t>(p)] != b) {
                        path_stamp[static_cast<std::size_t>(p)] = b;
                        path_sign[static_cast<std::size_t>(p)] = 0;
                        touched.push_back(p);
                    }
                    path_sign[static_cast<std::size_t>(p)] -= 1;
                }
                for (int p : capture_paths[static_cast<std::size_t>(nid)]) {
                    if (path_stamp[static_cast<std::size_t>(p)] != b) {
                        path_stamp[static_cast<std::size_t>(p)] = b;
                        path_sign[static_cast<std::size_t>(p)] = 0;
                        touched.push_back(p);
                    }
                    path_sign[static_cast<std::size_t>(p)] += 1;
                }
            }

            std::vector<AffectedPath> &out = branch_affected_paths[static_cast<std::size_t>(b)];
            out.reserve(touched.size());
            for (int p : touched) {
                const int sign = path_sign[static_cast<std::size_t>(p)];
                if (sign != 0)
                    out.push_back({p, sign});
            }
        }
    }

    const Clock::time_point t0 = Clock::now();
    int passes = 0;
    long long timing_evals = 0;

    std::multiset<double> setup_slacks;
    std::multiset<double> hold_slacks;
    double tns_setup_total = 0.0;
    double tns_hold_total = 0.0;
    double cur_area_total = 0.0;

    // Resync from the authoritative timing engine: rebuilds setup_slacks/hold_slacks/
    // tns_*_total/cur_area_total/cur_score from scratch. This bounds any incremental
    // floating-point drift from the trial/revert cycles below, and is far cheaper than doing a
    // full rescan per candidate (which is exactly the cost this rewrite exists to avoid -
    // O(n_paths) here, versus O(n_branches * n_candidates * n_paths) for a per-candidate scan).
    auto resync = [&]() {
        pd_annotate_clock(d);
        refresh_design_timing(d);
        setup_slacks.clear();
        hold_slacks.clear();
        tns_setup_total = 0.0;
        tns_hold_total = 0.0;
        for (int i = 0; i < d->n_paths; i++) {
            const PdPath &p = d->paths[i];
            if (p.launch_id < 0 || p.capture_id < 0) continue;
            setup_slacks.insert(p.slack_setup_ss);
            if (p.slack_setup_ss < 0.0) tns_setup_total += p.slack_setup_ss;
            hold_slacks.insert(p.slack_hold_ff);
            if (p.slack_hold_ff < 0.0) tns_hold_total += p.slack_hold_ff;
        }
        cur_area_total = d->total_area;
        cur_score = lp_compute_score(multiset_min_or_zero(setup_slacks), tns_setup_total,
                                     multiset_min_or_zero(hold_slacks), tns_hold_total,
                                     cur_area_total, pb->wns_ss_ori, pb->tns_ss_ori,
                                     pb->wns_ff_ori, pb->tns_ff_ori, pb->area_ori);
    };

    // Commits branch b to new_cell_idx for real: updates node->cell_idx/cur_ss/cur_ff, applies
    // the resulting slack/area deltas permanently, and refreshes cur_score.
    auto commit_branch = [&](int b, int new_cell_idx) {
        const LpBranch &br = pb->branches[static_cast<std::size_t>(b)];
        PdNode *node = &d->nodes[br.child_node];
        const int orig_cell_idx = node->cell_idx;
        if (orig_cell_idx == new_cell_idx)
            return;

        const std::vector<AffectedPath> &affected = branch_affected_paths[static_cast<std::size_t>(b)];
        const double orig_area = d->cells[orig_cell_idx].width * d->cells[orig_cell_idx].height;
        const PdCell *new_cell = &d->cells[new_cell_idx];
        const double delta_ss =
            lp_eval_branch_delay_ss(d, new_cell, br.fanout) - cur_ss[static_cast<std::size_t>(b)];
        const double delta_ff =
            lp_eval_branch_delay_ff(d, new_cell, br.fanout) - cur_ff[static_cast<std::size_t>(b)];
        const double new_area = new_cell->width * new_cell->height;

        apply_affected_delta(d, affected, delta_ss, delta_ff, &setup_slacks, &hold_slacks,
                             &tns_setup_total, &tns_hold_total);
        cur_area_total += new_area - orig_area;

        apply_cell_to_branch(d, br, new_cell_idx, &cur_ss, &cur_ff, b);
        // 確定要套用了，才花時間拷貝字串
        std::strncpy(node->cell, new_cell->name, PD_MAX_NAME - 1);
        node->cell[PD_MAX_NAME - 1] = '\0';

        cur_score = lp_compute_score(multiset_min_or_zero(setup_slacks), tns_setup_total,
                                     multiset_min_or_zero(hold_slacks), tns_hold_total,
                                     cur_area_total, pb->wns_ss_ori, pb->tns_ss_ori,
                                     pb->wns_ff_ori, pb->tns_ff_ori, pb->area_ori);
    };

    // Single-buffer hill-climb to a local optimum (or until `deadline`/wall_deadline hits),
    // visiting branches in `order` each pass - coordinate descent is order-dependent, so this
    // is parameterized to let different attempts explore different trajectories.
    auto converge = [&](const std::vector<int> &order, const Clock::time_point &deadline) {
        bool improved = true;
        while (improved && before_deadline(wall_deadline) && Clock::now() < deadline) {
            improved = false;
            passes++;
            resync();

            for (int b : order) {
                if (!before_deadline(wall_deadline)) break;
                const LpBranch &br = pb->branches[static_cast<std::size_t>(b)];
                if (br.kind != LpBranchKind::ExistingBuf) continue;

                PdNode *node = &d->nodes[br.child_node];
                if (node->kind != PD_NODE_BUF) continue;

                const int orig_cell_idx = node->cell_idx;
                const std::vector<int> &cands = branch_candidates[static_cast<std::size_t>(b)];
                if (cands.size() <= 1) continue;

                const std::vector<AffectedPath> &affected =
                    branch_affected_paths[static_cast<std::size_t>(b)];
                const double orig_area =
                    d->cells[orig_cell_idx].width * d->cells[orig_cell_idx].height;

                double best_delta = 0.0;
                int best_cell_idx = orig_cell_idx;

                for (int ci : cands) {
                    if (ci == orig_cell_idx) continue;
                    if (!before_deadline(wall_deadline)) break;

                    const PdCell *cell = &d->cells[ci];
                    const double delta_ss = lp_eval_branch_delay_ss(d, cell, br.fanout) -
                                            cur_ss[static_cast<std::size_t>(b)];
                    const double delta_ff = lp_eval_branch_delay_ff(d, cell, br.fanout) -
                                            cur_ff[static_cast<std::size_t>(b)];
                    const double new_area = cell->width * cell->height;

                    apply_affected_delta(d, affected, delta_ss, delta_ff, &setup_slacks,
                                         &hold_slacks, &tns_setup_total, &tns_hold_total);
                    timing_evals++;

                    const double cand_area = cur_area_total - orig_area + new_area;
                    const double cand_score = lp_compute_score(
                        multiset_min_or_zero(setup_slacks), tns_setup_total,
                        multiset_min_or_zero(hold_slacks), tns_hold_total, cand_area,
                        pb->wns_ss_ori, pb->tns_ss_ori, pb->wns_ff_ori, pb->tns_ff_ori,
                        pb->area_ori);
                    const double delta = cand_score - cur_score;
                    if (delta > best_delta + 1e-12) {
                        best_delta = delta;
                        best_cell_idx = ci;
                    }

                    // Undo the trial - restores setup_slacks/hold_slacks/tns_*_total exactly,
                    // so the next candidate is evaluated against the same unmodified baseline.
                    apply_affected_delta(d, affected, -delta_ss, -delta_ff, &setup_slacks,
                                         &hold_slacks, &tns_setup_total, &tns_hold_total);
                }

                if (best_delta > 1e-12) {
                    commit_branch(b, best_cell_idx);
                    improved = true;
                }
            }
        }
    };

    struct Snapshot {
        std::vector<int> cell_idx;
        double score = -1e30;
    };
    auto take_snapshot = [&]() {
        Snapshot s;
        s.cell_idx.assign(static_cast<std::size_t>(n_br), -1);
        for (int b = 0; b < n_br; b++) {
            const LpBranch &br = pb->branches[static_cast<std::size_t>(b)];
            if (br.kind == LpBranchKind::ExistingBuf)
                s.cell_idx[static_cast<std::size_t>(b)] = d->nodes[br.child_node].cell_idx;
        }
        s.score = cur_score;
        return s;
    };
    auto restore_snapshot = [&](const Snapshot &s) {
        for (int b = 0; b < n_br; b++) {
            const int ci = s.cell_idx[static_cast<std::size_t>(b)];
            if (ci < 0) continue;
            const LpBranch &br = pb->branches[static_cast<std::size_t>(b)];
            PdNode *node = &d->nodes[br.child_node];
            node->cell_idx = ci;
            std::strncpy(node->cell, d->cells[ci].name, PD_MAX_NAME - 1);
            node->cell[PD_MAX_NAME - 1] = '\0';
            cur_ss[static_cast<std::size_t>(b)] = lp_eval_branch_delay_ss(d, &d->cells[ci], br.fanout);
            cur_ff[static_cast<std::size_t>(b)] = lp_eval_branch_delay_ff(d, &d->cells[ci], br.fanout);
        }
        resync();
    };

    // Snapshot of the LP-init-synced (or baseline-reverted) starting point, before any
    // convergence attempt - every shuffled-order attempt below restarts from exactly here.
    const Snapshot start = take_snapshot();

    // Per-branch weight for the biased shuffle below: 1.0 base, +2.0 for every currently
    // violating path (at the `start` state - every shuffle attempt resets here, so this stays
    // valid for all of them) the branch touches. Mirrors sa_eval.cpp's sa_branch_weights, but
    // computed from branch_affected_paths (already built above) instead of a separate SaPgCtx.
    std::vector<double> branch_weight(static_cast<std::size_t>(n_br), 1.0);
    for (int b = 0; b < n_br; b++) {
        for (const AffectedPath &ap : branch_affected_paths[static_cast<std::size_t>(b)]) {
            const PdPath &p = d->paths[static_cast<std::size_t>(ap.path_idx)];
            if (p.slack_setup_ss < 0.0 || p.slack_hold_ff < 0.0)
                branch_weight[static_cast<std::size_t>(b)] += 2.0;
        }
    }

    // Weighted random permutation (Efraimidis-Spirakis): key_b = u_b^(1/w_b), sorted
    // descending. Reduces to a plain uniform shuffle when every weight is equal, and otherwise
    // biases higher-weight (currently-violating-path) branches toward the front of the order,
    // so hill-climbing reaches them before patience/deadline cuts an attempt short.
    auto weighted_shuffle = [&](std::vector<int> *ord, std::mt19937 &rng) {
        std::uniform_real_distribution<double> unif(1e-12, 1.0);
        std::vector<std::pair<double, int>> keyed(static_cast<std::size_t>(n_br));
        for (int b = 0; b < n_br; b++) {
            const double w = std::max(branch_weight[static_cast<std::size_t>(b)], 1e-6);
            keyed[static_cast<std::size_t>(b)] = {std::pow(unif(rng), 1.0 / w), b};
        }
        std::sort(keyed.begin(), keyed.end(),
                 [](const std::pair<double, int> &x, const std::pair<double, int> &y) {
                     return x.first > y.first;
                 });
        for (int i = 0; i < n_br; i++)
            (*ord)[static_cast<std::size_t>(i)] = keyed[static_cast<std::size_t>(i)].second;
    };

    std::vector<int> order(static_cast<std::size_t>(n_br));
    for (int i = 0; i < n_br; i++)
        order[static_cast<std::size_t>(i)] = i;

    // Fixed seed so the whole run (attempt 1 included) stays fully deterministic/reproducible.
    std::mt19937 rng(0xC0FFEEu);

    // Attempt 1 now also uses the weighted order rather than the plain tree (parent-before-
    // child) order: the natural order has no special convergence property here (coordinate
    // descent doesn't need a dependency-respecting order), it was only ever a deterministic
    // default. Time-starved testcases (e.g. huge ones where attempt 1 alone eats most of the
    // budget, leaving room for at most one more attempt) previously got *no* benefit from the
    // weighting until a later shuffle attempt happened to run - this way the very first attempt
    // already benefits from it too.
    weighted_shuffle(&order, rng);
    const Clock::time_point initial_deadline =
        t0 + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(time_limit_sec));
    converge(order, initial_deadline);
    Snapshot best = take_snapshot();
    int attempts = 1;
    int accepted_attempts = 0;

    // --- Shuffled-order attempts -------------------------------------------------------------
    // Coordinate-descent hill-climbing gets stuck at whatever local optimum the branch
    // processing order leads to - attempt 1's (weighted-random) order is just one trajectory
    // among many. Spend up to half of whatever time remains after the first attempt re-running
    // convergence from the same starting point with a freshly shuffled order each time, keeping
    // the best final result; the other half is deliberately left unused so main.cpp's dual-seed
    // safety net (trying the plain original design as an alternate starting point) still gets a
    // chance to run afterward. A patience limit avoids burning the whole budget on attempts that
    // keep finding nothing (e.g. a tiny already-optimal design).
    const double remain_call_budget = time_limit_sec - elapsed_sec(t0);
    const double remain_wall = std::chrono::duration<double>(wall_deadline - Clock::now()).count();
    const double shuffle_budget = 0.5 * std::max(0.0, std::min(remain_call_budget, remain_wall));
    const Clock::time_point shuffle_deadline =
        Clock::now() + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(shuffle_budget));

    constexpr int kPatience = 50;
    if (n_br > 1 && shuffle_budget > 0.5) {
        int attempts_since_improvement = 0;
        while (before_deadline(wall_deadline) && Clock::now() < shuffle_deadline &&
               attempts_since_improvement < kPatience) {
            attempts++;
            restore_snapshot(start);
            weighted_shuffle(&order, rng);
            converge(order, shuffle_deadline);

            if (cur_score > best.score + 1e-9) {
                best = take_snapshot();
                accepted_attempts++;
                attempts_since_improvement = 0;
            } else {
                attempts_since_improvement++;
            }
        }
    }
    restore_snapshot(best);

    pd_annotate_clock(d);
    refresh_design_timing(d);
    cur_score = score_design(pb, d);

    std::printf(
        "greedy_post_lp: finished after %d passes (%d order attempts, %d improved), %lld timing evals, %.1fs\n",
        passes, attempts, accepted_attempts, timing_evals, elapsed_sec(t0));

    lp_init->d_ss = cur_ss;
    lp_init->d_ff = cur_ff;

    LpMetrics greedy_m{};
    greedy_m.wns_setup_ss = d->wns_setup_ss;
    greedy_m.tns_setup_ss = d->tns_setup_ss;
    greedy_m.wns_hold_ff = d->wns_hold_ff;
    greedy_m.tns_hold_ff = d->tns_hold_ff;
    greedy_m.area = d->total_area;
    greedy_m.score = cur_score;

    char time_label[32];
    std::snprintf(time_label, sizeof(time_label), "%.1f", time_limit_sec);
    for (char *p = time_label; *p; ++p)
        if (*p == '.')
            *p = 'p';

    char basename[128];
    std::snprintf(basename, sizeof(basename), "greedy_postlp_bestimpr_t%s.txt", time_label);
    char path[1024];
    pd_join_path(path, sizeof(path), result_dir, basename);

    FILE *fp = std::fopen(path, "w");
    if (fp) {
        std::fprintf(fp, "greedy_postlp result\n");
        std::fprintf(fp, "testcase_dir: %s\n", testcase_dir);
        std::fprintf(fp, "time_limit_sec: %.1f\n", time_limit_sec);
        std::fprintf(fp, "lp_init_ok: 1\n");
        std::fprintf(fp, "greedy_elapsed_sec: %.3f\n", elapsed_sec(t0));
        std::fprintf(fp, "timing_evals: %lld\n", timing_evals);
        std::fprintf(fp, "solver: greedy_post_lp\n\n");

        std::fprintf(fp, "=== baseline (ori) ===\n");
        std::fprintf(fp, "SS setup WNS : %.6f  TNS : %.6f\n", pb->wns_ss_ori, pb->tns_ss_ori);
        std::fprintf(fp, "FF hold  WNS : %.6f  TNS : %.6f\n", pb->wns_ff_ori, pb->tns_ff_ori);
        std::fprintf(fp, "Total area   : %.6f\n", pb->area_ori);
        std::fprintf(fp, "Score (a=0.6, b=0.2, g=0.2): %.6f\n\n", 0.0);

        std::fprintf(fp, "=== after LP init ===\n");
        std::fprintf(fp, "SS setup WNS : %.6f  TNS : %.6f\n", lp_init_metrics->wns_setup_ss,
                     lp_init_metrics->tns_setup_ss);
        std::fprintf(fp, "FF hold  WNS : %.6f  TNS : %.6f\n", lp_init_metrics->wns_hold_ff,
                     lp_init_metrics->tns_hold_ff);
        std::fprintf(fp, "Total area   : %.6f\n", lp_init_metrics->area);
        std::fprintf(fp, "Score (a=0.6, b=0.2, g=0.2): %.6f\n\n", lp_init_metrics->score);

        std::fprintf(fp, "=== after greedy ===\n");
        std::fprintf(fp, "SS setup WNS : %.6f  TNS : %.6f\n", greedy_m.wns_setup_ss,
                     greedy_m.tns_setup_ss);
        std::fprintf(fp, "FF hold  WNS : %.6f  TNS : %.6f\n", greedy_m.wns_hold_ff,
                     greedy_m.tns_hold_ff);
        std::fprintf(fp, "Total area   : %.6f\n", greedy_m.area);
        std::fprintf(fp, "Score (a=0.6, b=0.2, g=0.2): %.6f\n", greedy_m.score);
        std::fclose(fp);
    }
    return 0;
}
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
#include <unordered_map>
#include <unordered_set>
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

static void recompute_path_slacks(PdDesign *d, int path_idx)
{
    PdPath *p = &d->paths[path_idx];
    const PdNode *launch = (p->launch_id >= 0) ? &d->nodes[p->launch_id] : nullptr;
    const PdNode *capture = (p->capture_id >= 0) ? &d->nodes[p->capture_id] : nullptr;
    double skew_ss = 0.0;
    double skew_ff = 0.0;

    if (launch && capture) {
        skew_ss = capture->d_clk_ss - launch->d_clk_ss;
        skew_ff = capture->d_clk_ff - launch->d_clk_ff;
    }

    p->slack_setup_ss = d->clock_period - d->t_setup - p->data_ss + skew_ss;
    p->slack_hold_ff = p->data_ff - d->t_hold - skew_ff;
}

static void recompute_timing_for_nodes(PdDesign *d, const std::unordered_set<int> &affected_nodes)
{
    int has_setup = 0;
    int has_hold = 0;

    d->wns_setup_ss = 1e30;
    d->tns_setup_ss = 0.0;
    d->wns_hold_ff = 1e30;
    d->tns_hold_ff = 0.0;

    for (int i = 0; i < d->n_paths; i++) {
        PdPath *p = &d->paths[i];
        if (affected_nodes.count(p->launch_id) || affected_nodes.count(p->capture_id))
            recompute_path_slacks(d, i);

        if (p->launch_id < 0 || p->capture_id < 0)
            continue;

        if (p->slack_setup_ss < d->wns_setup_ss)
            d->wns_setup_ss = p->slack_setup_ss;
        if (p->slack_setup_ss < 0.0)
            d->tns_setup_ss += p->slack_setup_ss;

        if (p->slack_hold_ff < d->wns_hold_ff)
            d->wns_hold_ff = p->slack_hold_ff;
        if (p->slack_hold_ff < 0.0)
            d->tns_hold_ff += p->slack_hold_ff;

        has_setup = 1;
        has_hold = 1;
    }

    if (!has_setup) {
        d->wns_setup_ss = 0.0;
        d->tns_setup_ss = 0.0;
    }
    if (!has_hold) {
        d->wns_hold_ff = 0.0;
        d->tns_hold_ff = 0.0;
    }
}

static void apply_cell_to_branch(PdDesign *d, const LpBranch &br, int cell_idx,
                                 std::vector<double> *cur_ss, std::vector<double> *cur_ff, int b)
{
    PdNode *node = &d->nodes[br.child_node];
    if (node->kind != PD_NODE_BUF)
        return;

    node->cell_idx = cell_idx;
    std::strncpy(node->cell, d->cells[cell_idx].name, PD_MAX_NAME - 1);
    node->cell[PD_MAX_NAME - 1] = '\0';
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

static void build_branch_candidates(const PdDesign *d_in, const LpProblem *pb,
                                    std::vector<std::vector<int>> *candidates)
{
    // We will create virtual combo cells by appending to a mutable copy
    // of the design. This allows representing 2-buffer series combos as
    // additional candidate "cells" while preserving the original library.
    PdDesign *d = const_cast<PdDesign *>(d_in);
    const int n_br = static_cast<int>(pb->branches.size());
    candidates->assign(static_cast<std::size_t>(n_br), {});

    for (int b = 0; b < n_br; b++) {
        const LpBranch &br = pb->branches[static_cast<std::size_t>(b)];
        if (br.kind != LpBranchKind::ExistingBuf)
            continue;

        struct Cand {
            int idx; // index into d->cells
            double ss;
            double ff;
            double area;
        };

        std::vector<Cand> cand_list;

        // Single-cell candidates (existing library cells)
        for (int ci = 0; ci < d->n_cells; ci++) {
            const PdCell *c = &d->cells[ci];
            if (br.fanout > c->max_fanout)
                continue;
            const double ss = lp_eval_branch_delay_ss(d, c, br.fanout);
            const double ff = lp_eval_branch_delay_ff(d, c, br.fanout);
            if (ss < br.d_ss_min - 1e-9 || ss > br.d_ss_max + 1e-9)
                continue;
            if (ff < br.d_ff_min - 1e-9 || ff > br.d_ff_max + 1e-9)
                continue;
            const double area = c->width * c->height;
            cand_list.push_back({ci, ss, ff, area});
        }

        // Two-buffer concatenations: first drives second (fanout 1), second drives branch fanout
        // Iterate over all pairs (i,j) and create virtual cells when within bounds.
        for (int i = 0; i < d->n_cells; i++) {
            const PdCell *c1 = &d->cells[i];
            // c1 drives only the second buffer => fanout 1
            if (1 > c1->max_fanout) continue;
            for (int j = 0; j < d->n_cells; j++) {
                const PdCell *c2 = &d->cells[j];
                if (br.fanout > c2->max_fanout) continue;

                const double ss = lp_eval_branch_delay_ss(d, c1, 1) + lp_eval_branch_delay_ss(d, c2, br.fanout);
                const double ff = lp_eval_branch_delay_ff(d, c1, 1) + lp_eval_branch_delay_ff(d, c2, br.fanout);
                if (ss < br.d_ss_min - 1e-9 || ss > br.d_ss_max + 1e-9) continue;
                if (ff < br.d_ff_min - 1e-9 || ff > br.d_ff_max + 1e-9) continue;

                const double area = c1->width * c1->height + c2->width * c2->height;

                // create a virtual PdCell entry if there is space
                if (d->n_cells + 1 < PD_MAX_CELLS) {
                    PdCell *v = &d->cells[d->n_cells];
                    char vname[PD_MAX_NAME];
                    std::snprintf(vname, sizeof(vname), "VCOMBO_%d_%d_b%d", i, j, b);
                    std::memset(v, 0, sizeof(*v));
                    std::strncpy(v->name, vname, PD_MAX_NAME - 1);
                    // store area in width*height (simple packing)
                    v->width = area;
                    v->height = 1.0;
                    v->max_fanout = std::max(1, br.fanout);
                    // fill delay table: for fanout f, delay = c1(1) + c2(f)
                    for (int f = 1; f <= v->max_fanout && f <= PD_MAX_FANOUT_TBL; f++) {
                        v->ss_delay[f - 1] = lp_eval_branch_delay_ss(d, c1, 1) + lp_eval_branch_delay_ss(d, c2, f);
                        v->ff_delay[f - 1] = lp_eval_branch_delay_ff(d, c1, 1) + lp_eval_branch_delay_ff(d, c2, f);
                    }
                    int vidx = d->n_cells;
                    d->n_cells++;
                    cand_list.push_back({vidx, ss, ff, area});
                }
            }
        }

        // Prune dominated candidates: if A.area >= B.area and A.ss >= B.ss and A.ff >= B.ff, drop A
        const double eps = 1e-12;
        std::vector<int> keep;
        for (size_t i = 0; i < cand_list.size(); i++) {
            bool dominated = false;
            for (size_t j = 0; j < cand_list.size(); j++) {
                if (i == j) continue;
                const Cand &A = cand_list[i];
                const Cand &B = cand_list[j];
                if (A.area >= B.area - eps && A.ss >= B.ss - eps && A.ff >= B.ff - eps) {
                    if (A.area > B.area + eps || A.ss > B.ss + eps || A.ff > B.ff + eps) {
                        dominated = true;
                        break;
                    }
                }
            }
            if (!dominated)
                keep.push_back(static_cast<int>(i));
        }

        std::vector<int> &out = (*candidates)[static_cast<std::size_t>(b)];
        out.reserve(keep.size());
        for (int k : keep)
            out.push_back(cand_list[static_cast<size_t>(k)].idx);

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
            if (br.fanout > c->max_fanout)
                continue;
            const double css = lp_eval_branch_delay_ss(d, c, br.fanout);
            const double err = std::fabs(css - target_ss);
            if (err < min_err) {
                min_err = err;
                best_ci = ci;
            }
        }

        if (best_ci >= 0)
            apply_cell_to_branch(d, br, best_ci, cur_ss, cur_ff, b);
    }
}

int greedy_post_lp(const char *result_dir, const char *testcase_dir, const LpProblem *pb,
                   const PdDesign *d_const, const LpBufferChainDp * /*dp_ss*/,
                   const LpBufferChainDp * /*dp_ff*/, LpSolution *lp_init,
                   const LpMetrics *lp_init_metrics, double time_limit_sec,
                   const Clock::time_point wall_deadline, char *err, std::size_t err_sz)
{
    std::printf("greedy_post_lp: entry (P0: deadline + pruned candidates + subtree timing)\n");

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

    std::printf("greedy_post_lp: budget %.1fs, wall remaining %.1fs\n", time_limit_sec,
                std::chrono::duration<double>(wall_deadline - Clock::now()).count());

    std::printf("greedy_post_lp: syncing graph to LP init...\n");
    sync_graph_from_delays(d, pb, &cur_ss, &cur_ff);

    pd_annotate_clock(d);
    refresh_design_timing(d);
    double cur_score = score_design(pb, d);

    if (cur_score < 0.0) {
        std::printf("greedy_post_lp: LP init score (%.6f) worse than baseline; reverting\n",
                    cur_score);
        for (int b = 0; b < n_br; b++) {
            const LpBranch &br = pb->branches[static_cast<std::size_t>(b)];
            if (br.kind != LpBranchKind::ExistingBuf || br.cell_idx < 0)
                continue;
            apply_cell_to_branch(d, br, br.cell_idx, &cur_ss, &cur_ff, b);
        }
        pd_annotate_clock(d);
        refresh_design_timing(d);
        cur_score = score_design(pb, d);
        const_cast<LpMetrics *>(lp_init_metrics)->wns_hold_ff = d->wns_hold_ff;
        const_cast<LpMetrics *>(lp_init_metrics)->tns_hold_ff = d->tns_hold_ff;
        std::printf("greedy_post_lp: revert complete, starting score %.6f\n", cur_score);
    }

    std::vector<std::vector<int>> branch_candidates;
    build_branch_candidates(d, pb, &branch_candidates);

    const Clock::time_point t0 = Clock::now();
    bool improved = true;
    int passes = 0;
    long long timing_evals = 0;

    while (improved && before_deadline(wall_deadline) &&
           elapsed_sec(t0) < time_limit_sec) {
        improved = false;
        passes++;

        for (int b = 0; b < n_br && before_deadline(wall_deadline); b++) {
            const LpBranch &br = pb->branches[static_cast<std::size_t>(b)];
            if (br.kind != LpBranchKind::ExistingBuf)
                continue;

            PdNode *node = &d->nodes[br.child_node];
            if (node->kind != PD_NODE_BUF)
                continue;

            const int orig_cell_idx = node->cell_idx;
            const std::vector<int> &cands = branch_candidates[static_cast<std::size_t>(b)];
            if (cands.size() <= 1)
                continue;

            double best_delta = 0.0;
            int best_cell_idx = orig_cell_idx;
            double best_score = cur_score;

            std::vector<int> subtree_nodes;
            collect_subtree_nodes(d, br.child_node, &subtree_nodes);
            std::unordered_set<int> affected_nodes;
            for (int nid : subtree_nodes) {
                const PdNode *n = &d->nodes[nid];
                if (n->kind == PD_NODE_FF)
                    affected_nodes.insert(nid);
            }

            for (int ci : cands) {
                if (ci == orig_cell_idx)
                    continue;
                if (!before_deadline(wall_deadline))
                    break;

                apply_cell_to_branch(d, br, ci, &cur_ss, &cur_ff, b);
                pd_annotate_clock_subtree(d, br.child_node);
                recompute_timing_for_nodes(d, affected_nodes);
                timing_evals++;

                const double cand_score = score_design(pb, d);
                const double delta = cand_score - cur_score;
                if (delta > best_delta + 1e-12) {
                    best_delta = delta;
                    best_cell_idx = ci;
                    best_score = cand_score;
                }
            }

            if (best_delta > 1e-12) {
                apply_cell_to_branch(d, br, best_cell_idx, &cur_ss, &cur_ff, b);
                pd_annotate_clock_subtree(d, br.child_node);
                recompute_timing_for_nodes(d, affected_nodes);
                cur_score = best_score;
                improved = true;
            } else {
                apply_cell_to_branch(d, br, orig_cell_idx, &cur_ss, &cur_ff, b);
                pd_annotate_clock_subtree(d, br.child_node);
                recompute_timing_for_nodes(d, affected_nodes);
            }
        }
    }

    pd_annotate_clock(d);
    refresh_design_timing(d);
    cur_score = score_design(pb, d);

    std::printf("greedy_post_lp: finished after %d passes, %lld timing evals, %.1fs\n", passes,
                timing_evals, elapsed_sec(t0));

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

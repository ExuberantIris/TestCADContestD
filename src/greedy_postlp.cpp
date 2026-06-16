#include "greedy_postlp.hpp"
#include "lp_score.hpp"
#include "pd_util.h"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <vector>

using Clock = std::chrono::steady_clock;

static double elapsed_sec(const Clock::time_point &t0)
{
    return std::chrono::duration<double>(Clock::now() - t0).count();
}

int greedy_post_lp(const char *result_dir, const char *testcase_dir, const LpProblem *pb,
                   const PdDesign *d_const, const LpBufferChainDp *dp_ss,
                   const LpBufferChainDp *dp_ff, LpSolution *lp_init,
                   const LpMetrics *lp_init_metrics, double time_limit_sec, char *err,
                   std::size_t err_sz)
{
    std::printf("greedy_post_lp: entry (Ultimate Reality Version)\n");

    if (!pb || !d_const || !dp_ss || !dp_ff || !lp_init || !lp_init_metrics || !result_dir) {
        if (err && err_sz > 0) std::snprintf(err, err_sz, "null arg");
        return -1;
    }

    PdDesign *d = const_cast<PdDesign *>(d_const);
    const double eps = 1e-9;
    const int n_br = static_cast<int>(pb->branches.size());
    std::vector<double> cur_ss = lp_init->d_ss;
    std::vector<double> cur_ff = lp_init->d_ff;

    if (cur_ss.empty() || cur_ff.empty()) return -1;

    // ---------------------------------------------------------
    // 🔧 步驟 1: 強制將圖形還原到 LP Init 的起點
    // ---------------------------------------------------------
    std::printf("greedy_post_lp: Syncing graph to reality...\n");
    for (int b = 0; b < n_br; b++) {
        const LpBranch &br = pb->branches[b];
        if (br.kind != LpBranchKind::ExistingBuf) continue;

        double target_ss = cur_ss[b];
        int best_ci = -1;
        double min_err = 1e9;
        
        // 找出 LP 建議的最接近真實元件
        for (int ci = 0; ci < d->n_cells; ci++) {
            const PdCell *c = &d->cells[ci];
            if (br.fanout > c->max_fanout) continue;
            double css = lp_eval_branch_delay_ss(d, c, br.fanout);
            double err = std::fabs(css - target_ss);
            if (err < min_err) {
                min_err = err;
                best_ci = ci;
            }
        }
        
        // 將該元件真實套用至圖形上
        if (best_ci >= 0) {
            PdNode *node = &d->nodes[br.child_node];
            if (node->kind == PD_NODE_BUF) {
                node->cell_idx = best_ci;
                std::strncpy(node->cell, d->cells[best_ci].name, PD_MAX_NAME - 1);
                node->cell[PD_MAX_NAME - 1] = '\0';
                cur_ss[b] = lp_eval_branch_delay_ss(d, &d->cells[best_ci], br.fanout);
                cur_ff[b] = lp_eval_branch_delay_ff(d, &d->cells[best_ci], br.fanout);
            }
        }
    }

    // 取得真正的初始分數
    pd_annotate_clock(d);
    pd_compute_timing(d);
    double cur_score = lp_compute_score(d->wns_setup_ss, d->tns_setup_ss, d->wns_hold_ff, d->tns_hold_ff, d->total_area,
                                        pb->wns_ss_ori, pb->tns_ss_ori, pb->wns_ff_ori, pb->tns_ff_ori, pb->area_ori);
    
    // =========================================================================
    // 🛡️ 拒絕 LP 毒藥：如果 LP 搞砸了 (真實分數 < 0)，直接時光倒流回 Baseline！
    // =========================================================================
    if (cur_score < 0.0) {
        std::printf("greedy_post_lp: LP Init score (%.6f) is worse than Baseline. Reverting to Baseline...\n", cur_score);
        for (int b = 0; b < n_br; b++) {
            const LpBranch &br = pb->branches[b];
            if (br.kind != LpBranchKind::ExistingBuf) continue;
            
            PdNode *node = &d->nodes[br.child_node];
            if (node->kind == PD_NODE_BUF) {
                // 還原為最原始的元件尺寸 (Baseline)
                node->cell_idx = br.cell_idx; 
                std::strncpy(node->cell, d->cells[br.cell_idx].name, PD_MAX_NAME - 1);
                node->cell[PD_MAX_NAME - 1] = '\0';
                
                // 更新當前的 delay 陣列，讓它跟圖形同步
                cur_ss[b] = lp_eval_branch_delay_ss(d, &d->cells[br.cell_idx], br.fanout);
                cur_ff[b] = lp_eval_branch_delay_ff(d, &d->cells[br.cell_idx], br.fanout);
            }
        }
        
        // 重新呼叫真實裁判計分 (此時分數應該會剛好歸零，等於 Baseline 的 0.0 分)
        pd_annotate_clock(d);
        pd_compute_timing(d);
        cur_score = lp_compute_score(d->wns_setup_ss, d->tns_setup_ss, d->wns_hold_ff, d->tns_hold_ff, d->total_area,
                                     pb->wns_ss_ori, pb->tns_ss_ori, pb->wns_ff_ori, pb->tns_ff_ori, pb->area_ori);
        std::printf("greedy_post_lp: Revert complete. New starting score: %.6f\n", cur_score);
        
        // 🚨 重要：更新傳入的 lp_init_metrics，這樣底下的 Hold-preserving 才會以 0 分的 Baseline 為防守基準
        const_cast<LpMetrics*>(lp_init_metrics)->wns_hold_ff = d->wns_hold_ff;
        const_cast<LpMetrics*>(lp_init_metrics)->tns_hold_ff = d->tns_hold_ff;
    }
    // =========================================================================

    const Clock::time_point t0 = Clock::now();
    bool improved = true;
    int passes = 0;

    // ---------------------------------------------------------
    // 🏃 步驟 2: 面對真實物理世界的 Greedy 迴圈
    // ---------------------------------------------------------
    while (improved && elapsed_sec(t0) < time_limit_sec) {
        improved = false;
        passes++;
        
        for (int b = 0; b < n_br && elapsed_sec(t0) < time_limit_sec; b++) {
            const LpBranch &br = pb->branches[b];
            if (br.kind != LpBranchKind::ExistingBuf) continue;

            PdNode *node = &d->nodes[br.child_node];
            if (node->kind != PD_NODE_BUF) continue;

            // 備份當前的圖形節點狀態
            int orig_cell_idx = node->cell_idx;
            char orig_cell_name[PD_MAX_NAME];
            std::strncpy(orig_cell_name, node->cell, PD_MAX_NAME);
            
            double best_delta = 0.0;
            int best_cell_idx = orig_cell_idx;
            double best_score = cur_score;

            // 暴力掃描所有真實元件
            for (int ci = 0; ci < d->n_cells; ci++) {
                if (ci == orig_cell_idx) continue; // 已經是這個元件就不用測了
                
                const PdCell *cand_c = &d->cells[ci];
                if (br.fanout > cand_c->max_fanout) continue;

                // 🔪 直接修改真實圖形
                node->cell_idx = ci;
                std::strncpy(node->cell, cand_c->name, PD_MAX_NAME - 1);
                node->cell[PD_MAX_NAME - 1] = '\0';

                // 呼叫真實裁判！
                pd_annotate_clock(d);
                pd_compute_timing(d);

                double cand_score = lp_compute_score(d->wns_setup_ss, d->tns_setup_ss, d->wns_hold_ff, d->tns_hold_ff, d->total_area,
                                                     pb->wns_ss_ori, pb->tns_ss_ori, pb->wns_ff_ori, pb->tns_ff_ori, pb->area_ori);

                // Hold-preserving 檢查 (不得比 LP_init 更糟)
                if (d->wns_hold_ff < lp_init_metrics->wns_hold_ff - eps ||
                    d->tns_hold_ff < lp_init_metrics->tns_hold_ff - eps) {
                    continue;
                }

                double delta = cand_score - cur_score;
                if (delta > best_delta + 1e-12) {
                    best_delta = delta;
                    best_cell_idx = ci;
                    best_score = cand_score;
                }
            }
            
            // 迴圈結束，決定這一步要走哪裡
            if (best_delta > 1e-12) {
                // 找到了更好的，套用最佳解
                node->cell_idx = best_cell_idx;
                std::strncpy(node->cell, d->cells[best_cell_idx].name, PD_MAX_NAME - 1);
                node->cell[PD_MAX_NAME - 1] = '\0';
                cur_score = best_score;
                
                // 順便把 delay 更新進 cur_ss/ff 以便傳回 main
                cur_ss[b] = lp_eval_branch_delay_ss(d, &d->cells[best_cell_idx], br.fanout);
                cur_ff[b] = lp_eval_branch_delay_ff(d, &d->cells[best_cell_idx], br.fanout);
                
                improved = true;
            } else {
                // 沒有找到更好的，還原回原本的元件
                node->cell_idx = orig_cell_idx;
                std::strncpy(node->cell, orig_cell_name, PD_MAX_NAME);
            }
        }
    }

    std::printf("greedy_post_lp: finished after %d passes.\n", passes);

    // 再次執行最後的真實時序計算，確保結果是最新的
    pd_annotate_clock(d);
    pd_compute_timing(d);

    // 回傳最佳解陣列給 main.cpp
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
    for (char *p = time_label; *p; ++p) if (*p == '.') *p = 'p';

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
        std::fprintf(fp, "solver: greedy_post_lp\n\n");

        std::fprintf(fp, "=== baseline (ori) ===\n");
        std::fprintf(fp, "SS setup WNS : %.6f  TNS : %.6f\n", pb->wns_ss_ori, pb->tns_ss_ori);
        std::fprintf(fp, "FF hold  WNS : %.6f  TNS : %.6f\n", pb->wns_ff_ori, pb->tns_ff_ori);
        std::fprintf(fp, "Total area   : %.6f\n", pb->area_ori);
        std::fprintf(fp, "Score (a=0.6, b=0.2, g=0.2): %.6f\n\n", 0.0);

        std::fprintf(fp, "=== after LP init ===\n");
        std::fprintf(fp, "SS setup WNS : %.6f  TNS : %.6f\n", lp_init_metrics->wns_setup_ss, lp_init_metrics->tns_setup_ss);
        std::fprintf(fp, "FF hold  WNS : %.6f  TNS : %.6f\n", lp_init_metrics->wns_hold_ff, lp_init_metrics->tns_hold_ff);
        std::fprintf(fp, "Total area   : %.6f\n", lp_init_metrics->area);
        std::fprintf(fp, "Score (a=0.6, b=0.2, g=0.2): %.6f\n\n", lp_init_metrics->score);

        std::fprintf(fp, "=== after greedy ===\n");
        std::fprintf(fp, "SS setup WNS : %.6f  TNS : %.6f\n", greedy_m.wns_setup_ss, greedy_m.tns_setup_ss);
        std::fprintf(fp, "FF hold  WNS : %.6f  TNS : %.6f\n", greedy_m.wns_hold_ff, greedy_m.tns_hold_ff);
        std::fprintf(fp, "Total area   : %.6f\n", greedy_m.area);
        std::fprintf(fp, "Score (a=0.6, b=0.2, g=0.2): %.6f\n", greedy_m.score);
        std::fclose(fp);
    }
    return 0;
}
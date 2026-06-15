#include "greedy_postlp.hpp"

#include "sa_eval.hpp"
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
                   const PdDesign *d, const LpBufferChainDp *dp_ss,
                   const LpBufferChainDp *dp_ff, const LpSolution *lp_init,
                   const LpMetrics *lp_init_metrics, double time_limit_sec, char *err,
                   std::size_t err_sz)
{
    // debug: write entry marker
    {
        char dbg[1024];
        if (pd_join_path(dbg, sizeof(dbg), result_dir, "greedy_debug.txt") == 0) {
            FILE *df = std::fopen(dbg, "a");
            if (df) {
                std::fprintf(df, "greedy_post_lp called\n");
                std::fclose(df);
            }
        }
    }

    std::printf("greedy_post_lp: entry\n");

    if (!pb || !d || !dp_ss || !dp_ff || !lp_init || !lp_init_metrics || !result_dir) {
        if (err && err_sz > 0)
            std::snprintf(err, err_sz, "null arg to greedy_post_lp");
        return -1;
    }

    const double eps = 1e-9;
    std::vector<BranchDpOpts> opts;
    sa_build_branch_opts(pb, const_cast<PdDesign *>(d), &opts);
    std::printf("greedy_post_lp: built %zu branch opts\n", opts.size());

    std::vector<double> cur_ss = lp_init->d_ss;
    std::vector<double> cur_ff = lp_init->d_ff;
    std::printf("greedy_post_lp: lp_init sizes ss=%zu ff=%zu\n", cur_ss.size(), cur_ff.size());
    if (cur_ss.empty() || cur_ff.empty()) {
        if (err && err_sz > 0)
            std::snprintf(err, err_sz, "LP init solution empty");
        return -1;
    }

    SaPgCtx ctx;
    if (!sa_build_ctx(pb, const_cast<PdDesign *>(d), &ctx)) {
        if (err && err_sz > 0)
            std::snprintf(err, err_sz, "sa_build_ctx failed in greedy_post_lp");
        return -1;
    }
    std::printf("greedy_post_lp: built ctx\n");
    std::printf("greedy_post_lp: before sa_eval_state\n");
    sa_eval_state(const_cast<LpProblem *>(pb), const_cast<PdDesign *>(d), cur_ss, cur_ff, dp_ss,
                  dp_ff, &ctx);
    std::printf("greedy_post_lp: after sa_eval_state\n");

    double cur_score = ctx.score;
    const Clock::time_point t0 = Clock::now();

    bool improved = true;
    int passes = 0;

    while (improved && elapsed_sec(t0) < time_limit_sec) {
        improved = false;
        passes++;
        const int n_br = static_cast<int>(pb->branches.size());
        for (int b = 0; b < n_br && elapsed_sec(t0) < time_limit_sec; b++) {
            const auto &o = opts[static_cast<std::size_t>(b)];

            double best_delta = 0.0;
            std::vector<double> best_ss = cur_ss;
            std::vector<double> best_ff = cur_ff;
            SaPgCtx best_ctx = ctx;

            // evaluate all ss candidates and find best
            for (double cand : o.ss_delays) {
                if (std::fabs(cand - cur_ss[static_cast<std::size_t>(b)]) < 1e-12)
                    continue;
                std::vector<double> trial_ss = cur_ss;
                std::vector<double> trial_ff = cur_ff;
                trial_ss[static_cast<std::size_t>(b)] = cand;
                SaPgCtx trial_ctx;
                trial_ctx.launch_ff = ctx.launch_ff;
                trial_ctx.capture_ff = ctx.capture_ff;
                trial_ctx.ff_path_br = ctx.ff_path_br;
                trial_ctx.ff_path_off = ctx.ff_path_off;
                trial_ctx.node_to_ff = ctx.node_to_ff;
                sa_eval_state(const_cast<LpProblem *>(pb), const_cast<PdDesign *>(d), trial_ss,
                              trial_ff, dp_ss, dp_ff, &trial_ctx);

                // hold-preserving vs LP_init
                if (trial_ctx.wns_ff < lp_init_metrics->wns_hold_ff - eps ||
                    trial_ctx.tns_ff < lp_init_metrics->tns_hold_ff - eps)
                    continue;

                double delta = trial_ctx.score - cur_score;
                if (delta > best_delta + 1e-12) {
                    best_delta = delta;
                    best_ss = std::move(trial_ss);
                    best_ff = std::move(trial_ff);
                    best_ctx = trial_ctx;
                }
            }

            // evaluate all ff candidates and find best
            for (double cand : o.ff_delays) {
                if (std::fabs(cand - cur_ff[static_cast<std::size_t>(b)]) < 1e-12)
                    continue;
                std::vector<double> trial_ss = cur_ss;
                std::vector<double> trial_ff = cur_ff;
                trial_ff[static_cast<std::size_t>(b)] = cand;
                SaPgCtx trial_ctx;
                trial_ctx.launch_ff = ctx.launch_ff;
                trial_ctx.capture_ff = ctx.capture_ff;
                trial_ctx.ff_path_br = ctx.ff_path_br;
                trial_ctx.ff_path_off = ctx.ff_path_off;
                trial_ctx.node_to_ff = ctx.node_to_ff;
                sa_eval_state(const_cast<LpProblem *>(pb), const_cast<PdDesign *>(d), trial_ss,
                              trial_ff, dp_ss, dp_ff, &trial_ctx);

                if (trial_ctx.wns_ff < lp_init_metrics->wns_hold_ff - eps ||
                    trial_ctx.tns_ff < lp_init_metrics->tns_hold_ff - eps)
                    continue;

                double delta = trial_ctx.score - cur_score;
                if (delta > best_delta + 1e-12) {
                    best_delta = delta;
                    best_ss = std::move(trial_ss);
                    best_ff = std::move(trial_ff);
                    best_ctx = trial_ctx;
                }
            }

            // accept best move if found
            if (best_delta > 1e-12) {
                cur_ss = std::move(best_ss);
                cur_ff = std::move(best_ff);
                ctx = best_ctx;
                cur_score = ctx.score;
                improved = true;
            }
        }
    }

    // prepare metrics
    LpMetrics greedy_m{};
    greedy_m.wns_setup_ss = ctx.wns_ss;
    greedy_m.tns_setup_ss = ctx.tns_ss;
    greedy_m.wns_hold_ff = ctx.wns_ff;
    greedy_m.tns_hold_ff = ctx.tns_ff;
    greedy_m.area = ctx.area;
    greedy_m.score = ctx.score;

    // write out result_postlp_greedy_<iters>_t<time>.txt in result_dir
    char time_label[32];
    if (std::fabs(time_limit_sec - std::round(time_limit_sec)) < 1e-6)
        std::snprintf(time_label, sizeof(time_label), "%.0f", time_limit_sec);
    else
        std::snprintf(time_label, sizeof(time_label), "%.1f", time_limit_sec);
    for (char *p = time_label; *p; ++p) {
        if (*p == '.')
            *p = 'p';
    }

    char basename[128];
    std::snprintf(basename, sizeof(basename), "greedy_postlp_bestimpr_t%s.txt", time_label);
    char path[1024];
    if (pd_join_path(path, sizeof(path), result_dir, basename) != 0) {
        if (err && err_sz > 0)
            std::snprintf(err, err_sz, "result path too long");
        return -1;
    }

    int suffix = 1;
    while (true) {
        FILE *f = std::fopen(path, "r");
        if (!f)
            break;
        std::fclose(f);
        if (std::snprintf(basename, sizeof(basename), "greedy_postlp_bestimpr_t%s_%d.txt",
                          time_label, suffix) >= (int)sizeof(basename)) {
            if (err && err_sz > 0)
                std::snprintf(err, err_sz, "result basename too long");
            return -1;
        }
        if (pd_join_path(path, sizeof(path), result_dir, basename) != 0) {
            if (err && err_sz > 0)
                std::snprintf(err, err_sz, "result path too long");
            return -1;
        }
        suffix++;
    }

    FILE *fp = std::fopen(path, "w");
    if (!fp) {
        if (err && err_sz > 0)
            std::snprintf(err, err_sz, "cannot open %s for write", path);
        return -1;
    }

    std::fprintf(fp, "greedy_postlp result\n");
    std::fprintf(fp, "testcase_dir: %s\n", testcase_dir);
    std::fprintf(fp, "time_limit_sec: %.1f\n", time_limit_sec);
    std::fprintf(fp, "lp_init_ok: 1\n");
    std::fprintf(fp, "greedy_elapsed_sec: %.3f\n", elapsed_sec(t0));
    std::fprintf(fp, "greedy_elapsed_time: %.3f sec\n", elapsed_sec(t0));
    std::fprintf(fp, "solver: greedy_post_lp\n");
    std::fprintf(fp, "solver_status: 0\n\n");

    std::fprintf(fp, "=== baseline (ori) ===\n");
    // baseline available from pb via ori-like values
    std::fprintf(fp, "SS setup WNS : %.6f  TNS : %.6f\n", pb->wns_ss_ori, pb->tns_ss_ori);
    std::fprintf(fp, "FF hold  WNS : %.6f  TNS : %.6f\n", pb->wns_ff_ori, pb->tns_ff_ori);
    std::fprintf(fp, "Total area   : %.6f\n", pb->area_ori);
    std::fprintf(fp, "Score (a=b=g=1): %.6f\n\n", 0.0);

    std::fprintf(fp, "=== after LP init ===\n");
    std::fprintf(fp, "SS setup WNS : %.6f  TNS : %.6f\n", lp_init_metrics->wns_setup_ss,
                    lp_init_metrics->tns_setup_ss);
    std::fprintf(fp, "FF hold  WNS : %.6f  TNS : %.6f\n", lp_init_metrics->wns_hold_ff,
                    lp_init_metrics->tns_hold_ff);
    std::fprintf(fp, "Total area   : %.6f\n", lp_init_metrics->area);
    std::fprintf(fp, "Score (a=b=g=1): %.6f\n\n", lp_init_metrics->score);

    std::fprintf(fp, "=== after greedy ===\n");
    std::fprintf(fp, "SS setup WNS : %.6f  TNS : %.6f\n", greedy_m.wns_setup_ss, greedy_m.tns_setup_ss);
    std::fprintf(fp, "FF hold  WNS : %.6f  TNS : %.6f\n", greedy_m.wns_hold_ff, greedy_m.tns_hold_ff);
    std::fprintf(fp, "Total area   : %.6f\n", greedy_m.area);
    std::fprintf(fp, "Score (a=b=g=1): %.6f\n", greedy_m.score);

    std::fclose(fp);
    return 0;
}

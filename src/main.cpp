#include "lp_buffer_dp.hpp"
#include "lp_mo_init.hpp"
#include "lp_score.hpp"
#include "lp_types.hpp"
#include "sa_apply.hpp"
#include "sa_eval.hpp"
// #include "sa_path_solve.hpp" // ✂️ 移除 SA 標頭檔
#include "greedy_postlp.hpp"
#include "sa_solve.hpp"

#include <chrono>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <sys/stat.h>
#include <cmath>

namespace {

constexpr double kTailReserveSec = 5.0;

using SteadyClock = std::chrono::steady_clock;

static double remaining_wall_sec(const SteadyClock::time_point &deadline)
{
    return std::chrono::duration<double>(deadline - SteadyClock::now()).count();
}

} // namespace

static void read_time_limit(LpProblem *pb)
{
    const char *env = std::getenv("SA_TIME_LIMIT");
    if (!env || !env[0])
        env = std::getenv("LP_TIME_LIMIT");
    if (env && env[0]) {
        const double t = std::atof(env);
        if (t > 0.1)
            pb->time_limit_sec = t;
    }
}

static double read_lp_init_limit()
{
    const char *env = std::getenv("LP_INIT_TIME_LIMIT");
    if (env && env[0]) {
        const double t = std::atof(env);
        if (t > 0.1)
            return t;
    }
    return 15.0;
}

static double read_greedy_time_limit()
{
    const char *env = std::getenv("GREEDY_TIME_LIMIT");
    if (env && env[0]) {
        const double t = std::atof(env);
        if (t > 0.1)
            return t;
    }
    return 540.0; /* 9 min */
}

int main(int argc, char **argv)
{
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    PdDesign design{};
    LpProblem problem;
    SaSolveResult sa_result{}; // 保留這個 struct 用來當作資料載體傳給 output
    LpSolution lp_init{};
    LpMetrics ori{}, opt{};
    LpMetrics lp_init_metrics{};
    LpBufferChainDp dp_ss, dp_ff;
    char err[512];
    const char *testcase_dir;
    const auto wall_t0 = std::chrono::steady_clock::now();

    if (argc < 2) {
        std::fprintf(stderr, "Usage: %s <testcase_dir> [result_dir]\n", argv[0]);
        return 1;
    }

    testcase_dir = argv[1];
    lp_problem_init(&problem);
    read_time_limit(&problem);
    if (problem.time_limit_sec <= 0.0)
        problem.time_limit_sec = 600.0;

    const double total_limit = problem.time_limit_sec;
    const double lp_init_limit = read_lp_init_limit();
    const double greedy_time_limit = read_greedy_time_limit();
    const auto wall_deadline =
        wall_t0 + std::chrono::duration_cast<SteadyClock::duration>(
                      std::chrono::duration<double>(std::max(0.0, total_limit - kTailReserveSec)));

    const double lp_budget =
        std::min(lp_init_limit, std::max(0.0, remaining_wall_sec(wall_deadline)));
    const double greedy_budget_at_start =
        std::min(greedy_time_limit, std::max(0.0, remaining_wall_sec(wall_deadline)));

    std::printf("=== sa_solver (Greedy Focus Version) ===\n");
    std::printf("Input folder: %s\n", testcase_dir);
    std::printf("Total limit : %.1f sec | LP init: %.1f sec (budget %.1f) | Greedy cap: %.1f sec\n",
                total_limit, lp_init_limit, lp_budget, greedy_time_limit);
    std::printf("Wall deadline: %.1f sec (reserve %.1f sec for output)\n",
                std::chrono::duration<double>(wall_deadline - wall_t0).count(), kTailReserveSec);

    if (pd_load_design(testcase_dir, &design, err, sizeof(err)) != 0) {
        std::fprintf(stderr, "Load failed: %s\n", err);
        return 1;
    }

    if (lp_build_from_design(&problem, &design, err, sizeof(err)) != 0) {
        std::fprintf(stderr, "Problem build failed: %s\n", err);
        pd_free_design(&design);
        return 1;
    }

    double dp_max_delay = 0.0;
    for (const LpBranch &br : problem.branches) {
        dp_max_delay = std::max(dp_max_delay, br.d_ss_max);
        dp_max_delay = std::max(dp_max_delay, br.d_ff_max);
    }
    dp_max_delay = std::min(LpBufferChainDp::kMaxDelay, dp_max_delay + 0.02);
    std::printf("DP max delay: %.4f\n", dp_max_delay);

    if (dp_ss.build(&design, LpBufferDpCorner::SS, dp_max_delay) != 0 ||
        dp_ff.build(&design, LpBufferDpCorner::FF, dp_max_delay) != 0) {
        std::fprintf(stderr, "DP table build failed\n");
        pd_free_design(&design);
        return 1;
    }

    lp_compute_metrics(&design, &ori);
    ori.score = lp_compute_score(ori.wns_setup_ss, ori.tns_setup_ss, ori.wns_hold_ff, ori.tns_hold_ff,
                                 ori.area, problem.wns_ss_ori, problem.tns_ss_ori, problem.wns_ff_ori,
                                 problem.tns_ff_ori, problem.area_ori);
    lp_print_metrics("baseline (ori)", &ori);

    std::vector<BranchDpOpts> opts;
    sa_build_branch_opts(&problem, &design, &opts);
    
    // 雖然不跑 SA 了，但我們保留 initial 作為沒有解時的 fallback
    LpSolution initial;
    sa_init_from_design(&problem, &design, opts, &initial.d_ss, &initial.d_ff);

    const auto lp_t0 = std::chrono::steady_clock::now();
    
    // ---------------------------------------------------------
    // Phase 1: LP Init
    // ---------------------------------------------------------
    if (lp_solve_mo_init(&problem, &design, &lp_init, lp_budget, err, sizeof(err)) == 0 &&
        !lp_init.d_ss.empty()) {
        initial.d_ss = lp_init.d_ss;
        initial.d_ff = lp_init.d_ff;
        sa_result.lp_init_ok = 1;
        std::printf("LP init: %s (status=%d)\n", lp_init.solver_name.c_str(), lp_init.status);
        {
            SaPgCtx lp_ctx;
            if (sa_build_ctx(&problem, &design, &lp_ctx)) {
                sa_eval_state(&problem, &design, lp_init.d_ss, lp_init.d_ff, &dp_ss, &dp_ff, &lp_ctx);
                lp_init_metrics.wns_setup_ss = lp_ctx.wns_ss;
                lp_init_metrics.tns_setup_ss = lp_ctx.tns_ss;
                lp_init_metrics.wns_hold_ff = lp_ctx.wns_ff;
                lp_init_metrics.tns_hold_ff = lp_ctx.tns_ff;
                lp_init_metrics.area = lp_ctx.area;
                lp_init_metrics.score = lp_ctx.score;
            }
        }
    } else {
        sa_result.lp_init_ok = 0;
        std::printf("LP init: skipped/failed, using original clock tree delays\n");
    }
    sa_result.lp_init_sec = std::chrono::duration<double>(std::chrono::steady_clock::now() - lp_t0).count();
// 🔪 強制將 LP 初始解轉換為真實圖形，並請出真實裁判計分
    if (sa_result.lp_init_ok) {
        for (std::size_t b = 0; b < problem.branches.size(); b++) {
            const LpBranch &br = problem.branches[b];
            
            // 消除幽靈 Buffer
            if (br.kind != LpBranchKind::ExistingBuf) {
                lp_init.d_ss[b] = 0.0; // 🌟 修正：改成 lp_init
                lp_init.d_ff[b] = 0.0; // 🌟 修正：改成 lp_init
                continue;
            }

            // 尋找真實物理元件
            double target_ss = lp_init.d_ss[b]; // 🌟 修正：改成 lp_init
            int best_ci = -1;
            double min_err = 1e9;
            for (int ci = 0; ci < design.n_cells; ci++) {
                if (br.fanout > design.cells[ci].max_fanout) continue;
                double css = lp_eval_branch_delay_ss(&design, &design.cells[ci], br.fanout);
                double err = std::fabs(css - target_ss);
                if (err < min_err) {
                    min_err = err;
                    best_ci = ci;
                }
            }

            // 直接把真實元件塞進電路圖
            if (best_ci >= 0) {
                PdNode *node = &design.nodes[br.child_node];
                if (node->kind == PD_NODE_BUF) {
                    node->cell_idx = best_ci;
                    std::strncpy(node->cell, design.cells[best_ci].name, PD_MAX_NAME - 1);
                    node->cell[PD_MAX_NAME - 1] = '\0';
                }
            }
        }

        // 呼叫真實的時序引擎，算出沒有幻覺的分數！
        pd_annotate_clock(&design);
        pd_compute_timing(&design);

        lp_init_metrics.wns_setup_ss = design.wns_setup_ss;
        lp_init_metrics.tns_setup_ss = design.tns_setup_ss;
        lp_init_metrics.wns_hold_ff  = design.wns_hold_ff;
        lp_init_metrics.tns_hold_ff  = design.tns_hold_ff;
        lp_init_metrics.area         = design.total_area;
        lp_init_metrics.score        = lp_compute_score(
            design.wns_setup_ss, design.tns_setup_ss, design.wns_hold_ff, design.tns_hold_ff, design.total_area,
            problem.wns_ss_ori, problem.tns_ss_ori, problem.wns_ff_ori, problem.tns_ff_ori, problem.area_ori
        );
    }
    // ---------------------------------------------------------
    // Phase 2: Greedy Local Search
    // ---------------------------------------------------------
    if (sa_result.lp_init_ok) {
        const double greedy_budget =
            std::min(greedy_time_limit, std::max(0.0, remaining_wall_sec(wall_deadline)));
        std::printf("Running greedy_post_lp (budget %.1fs, wall remaining %.1fs)...\n",
                    greedy_budget, remaining_wall_sec(wall_deadline));
        if (greedy_budget > 0.1 &&
            greedy_post_lp(argv[2], testcase_dir, &problem, &design, &dp_ss, &dp_ff, &lp_init,
                           &lp_init_metrics, greedy_budget, wall_deadline, err,
                           sizeof(err)) == 0) {
            std::printf("Greedy optimization finished successfully.\n");
            
            // 將 Greedy 算出來的結果裝進最終要輸出的結構裡
            sa_result.solution.d_ss = lp_init.d_ss;
            sa_result.solution.d_ff = lp_init.d_ff;
            sa_result.solution.status = 1;
            sa_result.solution.solver_name = "Greedy_Best_Impr";
        } else if (greedy_budget <= 0.1) {
            std::printf("Greedy skipped: no wall time remaining\n");
            sa_result.solution = lp_init;
        } else {
            std::fprintf(stderr, "Greedy post-LP failed: %s\n", err);
            sa_result.solution = lp_init;
        }
    } else {
        sa_result.solution = initial;
    }
    sa_result.elapsed_sec = std::chrono::duration<double>(std::chrono::steady_clock::now() - lp_t0).count();

    // ✂️ 在這裡，我們把原本呼叫 sa_path_solve 的一大段程式碼全部砍掉了！

    // ---------------------------------------------------------
    // Phase 3: Apply & Output
    // ---------------------------------------------------------
    std::printf("Solver: %s (status=%d, lp=%.1fs, total_elapsed=%.1fs)\n",
                sa_result.solution.solver_name.c_str(), sa_result.solution.status,
                sa_result.lp_init_sec, sa_result.elapsed_sec);

    if (sa_apply_solution(&design, &problem, &sa_result.solution, &dp_ss, &dp_ff, err,
                          sizeof(err)) != 0) {
        std::fprintf(stderr, "Apply failed: %s\n", err);
        // sa_solution_free(&sa_result);
        lp_problem_free(&problem);
        pd_free_design(&design);
        return 1;
    }

    lp_compute_metrics(&design, &opt);
    opt.score = lp_compute_score(opt.wns_setup_ss, opt.tns_setup_ss, opt.wns_hold_ff, opt.tns_hold_ff,
                                 opt.area, problem.wns_ss_ori, problem.tns_ss_ori, problem.wns_ff_ori,
                                 problem.tns_ff_ori, problem.area_ori);
    lp_print_metrics("after optimize", &opt);

    const double wall_elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - wall_t0).count();

    if (argc >= 3) {
        char struct_path[1024];
        mkdir(argv[2], 0755);

        // 寫入 result.txt
        if (lp_write_result_txt(argv[2], testcase_dir, &ori, &lp_init_metrics, &opt,
                                sa_result.solution.solver_name.c_str(), sa_result.solution.status,
                                total_limit, greedy_budget_at_start, sa_result.lp_init_sec,
                                sa_result.lp_init_ok, sa_result.elapsed_sec, wall_elapsed,
                                sa_result.iterations, sa_result.use_second_best, err,
                                sizeof(err)) != 0) {
            std::fprintf(stderr, "Write result.txt failed: %s\n", err);
        } else {
            char result_txt[1024];
            if (pd_join_path(result_txt, sizeof(result_txt), argv[2], "result.txt") == 0)
                std::printf("Wrote %s\n", result_txt);
        }

        // 寫入 modified_clk_tree.structure
        if (pd_join_path(struct_path, sizeof(struct_path), argv[2],
                         "modified_clk_tree.structure") != 0) {
            std::fprintf(stderr, "Output path too long\n");
        } else if (pd_write_structure(&design, struct_path, err, sizeof(err)) != 0) {
            std::fprintf(stderr, "Write structure failed: %s\n", err);
        } else {
            std::printf("Wrote %s\n", struct_path);
        }
    }

    // sa_solution_free(&sa_result);
    lp_problem_free(&problem);
    pd_free_design(&design);
    return 0;
}
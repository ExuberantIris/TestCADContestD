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

    std::printf("=== sa_solver (Greedy Focus Version) ===\n");
    std::printf("Input folder: %s\n", testcase_dir);
    std::printf("Total limit : %.1f sec | LP init: %.1f sec | Greedy: %.1f sec\n",
                total_limit, lp_init_limit, greedy_time_limit);

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
    if (lp_solve_mo_init(&problem, &design, &lp_init, lp_init_limit, err, sizeof(err)) == 0 &&
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
    if (sa_result.lp_init_ok) {
        for (std::size_t b = 0; b < problem.branches.size(); b++) {
            if (problem.branches[b].kind != LpBranchKind::ExistingBuf) {
                lp_init.d_ss[b] = 0.0;
                lp_init.d_ff[b] = 0.0;
            }
        }
        // 重新評估真正的 LP Init 現實分數
        SaPgCtx reality_ctx;
        sa_build_ctx(&problem, &design, &reality_ctx);
        sa_eval_state(&problem, &design, lp_init.d_ss, lp_init.d_ff, &dp_ss, &dp_ff, &reality_ctx);
        lp_init_metrics.wns_setup_ss = reality_ctx.wns_ss;
        lp_init_metrics.tns_setup_ss = reality_ctx.tns_ss;
        lp_init_metrics.wns_hold_ff = reality_ctx.wns_ff;
        lp_init_metrics.tns_hold_ff = reality_ctx.tns_ff;
        lp_init_metrics.area = reality_ctx.area;
        lp_init_metrics.score = reality_ctx.score;
    }
    // ---------------------------------------------------------
    // Phase 2: Greedy Local Search
    // ---------------------------------------------------------
    if (sa_result.lp_init_ok) {
        std::printf("Running greedy_post_lp (hold-preserving)...\n");
        // 注意：待會我們改 greedy_postlp.cpp 時，要把 lp_init 的 const 拿掉
        // 讓 greedy 可以把算出來的最佳解直接寫回 lp_init 裡面
        if (greedy_post_lp(argv[2], testcase_dir, &problem, &design, &dp_ss, &dp_ff,
                           &lp_init, &lp_init_metrics, greedy_time_limit, err,
                           sizeof(err)) == 0) {
            std::printf("Greedy optimization finished successfully.\n");
            
            // 將 Greedy 算出來的結果裝進最終要輸出的結構裡
            sa_result.solution.d_ss = lp_init.d_ss;
            sa_result.solution.d_ff = lp_init.d_ff;
            sa_result.solution.status = 1;
            sa_result.solution.solver_name = "Greedy_Best_Impr";
        } else {
            std::fprintf(stderr, "Greedy post-LP failed: %s\n", err);
            // 失敗就 fallback 到 LP 的解
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
                                total_limit, greedy_time_limit, sa_result.lp_init_sec,
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
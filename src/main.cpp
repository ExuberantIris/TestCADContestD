#include "lp_buffer_dp.hpp"
#include "lp_score.hpp"
#include "lp_types.hpp"
#include "pd_output.h"
#include "sa_apply.hpp"
#include "sa_eval.hpp"
#include "sa_params.hpp"
#include "sa_solve.hpp"
#include "setup_lp_solve.hpp"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <sys/stat.h>
#include <vector>

namespace {

using Clock = std::chrono::steady_clock;

double elapsed_sec(const Clock::time_point &t0)
{
    return std::chrono::duration<double>(Clock::now() - t0).count();
}

void metrics_from_ctx(const LpProblem *pb, const SaPgCtx &ctx, const LpScoreWeights &wt,
                      LpMetrics *m)
{
    m->wns_setup_ss = ctx.wns_ss;
    m->tns_setup_ss = ctx.tns_ss;
    m->wns_hold_ff = ctx.wns_ff;
    m->tns_hold_ff = ctx.tns_ff;
    m->area = ctx.area;
    m->score = lp_compute_weighted_score(ctx.wns_ss, ctx.tns_ss, ctx.wns_ff, ctx.tns_ff, ctx.area,
                                         pb->wns_ss_ori, pb->tns_ss_ori, pb->wns_ff_ori,
                                         pb->tns_ff_ori, pb->area_ori, wt);
}

} // namespace

int main(int argc, char **argv)
{
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    const Clock::time_point wall_t0 = Clock::now();

    PdDesign design{};
    LpProblem problem;
    SaSolveResult phase1{};
    SaSolveResult phase2{};
    LpMetrics ori{}, phase1_metrics{}, phase2_metrics{};
    LpBufferChainDp dp_ss, dp_ff;
    SaParams sa_params{};
    char err[512];
    const char *testcase_dir;

    if (argc < 2) {
        std::fprintf(stderr, "Usage: %s <testcase_dir> [result_dir]\n", argv[0]);
        return 1;
    }

    testcase_dir = argv[1];
    sa_params_load(&sa_params, kSaParamsPath);

    lp_problem_init(&problem);
    problem.time_limit_sec = sa_params.total_time_limit_sec;
    if (problem.time_limit_sec <= 0.0)
        problem.time_limit_sec = 600.0;

    const double total_limit = problem.time_limit_sec;
    const double sa_limit =
        sa_params.sa_time_limit_sec > 0.0 ? sa_params.sa_time_limit_sec : 180.0;
    const LpScoreWeights &wt = sa_params.score_weights;

    std::printf("=== sa_solver (seg-tree direct + SA) ===\n");
    std::printf("Input folder: %s\n", testcase_dir);
    std::printf("Params file : %s\n", kSaParamsPath);
    std::printf("Total limit : %.1f sec | SA phase: %.1f sec\n", total_limit, sa_limit);
    std::printf("Score weights: a=%.4f b=%.4f g=%.4f | no_improve_limit=%d | sa_batch_size=%d\n",
                wt.a, wt.b, wt.g, sa_params.no_improve_limit, sa_params.sa_batch_size);

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
    dp_max_delay = std::min(LpBufferChainDp::kMaxDelay, dp_max_delay + sa_params.dp_delay_margin);
    std::printf("DP max delay: %.4f\n", dp_max_delay);

    if (dp_ss.build(&design, LpBufferDpCorner::SS, dp_max_delay) != 0 ||
        dp_ff.build(&design, LpBufferDpCorner::FF, dp_max_delay) != 0) {
        std::fprintf(stderr, "DP table build failed\n");
        pd_free_design(&design);
        return 1;
    }

    lp_compute_metrics(&design, &ori);
    ori.score = lp_compute_weighted_score(ori.wns_setup_ss, ori.tns_setup_ss, ori.wns_hold_ff,
                                          ori.tns_hold_ff, ori.area, problem.wns_ss_ori,
                                          problem.tns_ss_ori, problem.wns_ff_ori, problem.tns_ff_ori,
                                          problem.area_ori, wt);
    lp_print_weighted_metrics("baseline (ori)", &ori, wt);

    std::vector<double> ori_ff_clk(static_cast<std::size_t>(design.n_nodes), 0.0);
    for (int i = 0; i < design.n_nodes; i++) {
        if (design.nodes[i].kind == PD_NODE_FF)
            ori_ff_clk[static_cast<std::size_t>(i)] = design.nodes[i].d_clk_ss;
    }

    std::vector<BranchDpOpts> opts;
    sa_build_branch_opts(&problem, &design, &opts);
    LpSolution initial;
    sa_init_from_design(&problem, &design, opts, &initial.d_ss, &initial.d_ff);

    if (setup_longest_path_solve(&problem, &design, &dp_ss, &dp_ff, &initial, sa_limit, &phase1,
                                 err, sizeof(err)) != 0) {
        std::fprintf(stderr, "Phase-1 seg-tree failed: %s\n", err);
        sa_solution_free(&phase1);
        sa_solution_free(&phase2);
        lp_problem_free(&problem);
        pd_free_design(&design);
        return 1;
    }

    std::printf("Phase-1: %s (status=%d, iters=%lld, elapsed=%.1fs)\n",
                phase1.solution.solver_name.c_str(), phase1.solution.status,
                static_cast<long long>(phase1.iterations), phase1.elapsed_sec);

    SaPgCtx phase1_ctx;
    if (sa_build_ctx(&problem, &design, &phase1_ctx)) {
        sa_eval_state(&problem, &design, phase1.solution.d_ss, phase1.solution.d_ff, &dp_ss, &dp_ff,
                      &phase1_ctx);
        metrics_from_ctx(&problem, phase1_ctx, wt, &phase1_metrics);
        lp_print_weighted_metrics("after phase-1 (seg_tree_direct)", &phase1_metrics, wt);
    }

    const double elapsed_before_sa = elapsed_sec(wall_t0);
    const double sa_budget =
        std::min(sa_limit, std::max(0.0, total_limit - elapsed_before_sa));

    if (seg_tree_sa_solve(&problem, &design, &dp_ss, &dp_ff, &phase1.solution, ori_ff_clk,
                          sa_params, sa_budget, &phase2, err, sizeof(err)) != 0) {
        std::fprintf(stderr, "Phase-2 SA failed: %s\n", err);
        sa_solution_free(&phase1);
        sa_solution_free(&phase2);
        lp_problem_free(&problem);
        pd_free_design(&design);
        return 1;
    }

    std::printf("Phase-2: %s (status=%d, iters=%lld, elapsed=%.1fs)\n",
                phase2.solution.solver_name.c_str(), phase2.solution.status,
                static_cast<long long>(phase2.iterations), phase2.elapsed_sec);

    SaPgCtx phase2_ctx;
    if (sa_build_ctx(&problem, &design, &phase2_ctx)) {
        sa_eval_state(&problem, &design, phase2.solution.d_ss, phase2.solution.d_ff, &dp_ss, &dp_ff,
                      &phase2_ctx);
        metrics_from_ctx(&problem, phase2_ctx, wt, &phase2_metrics);
        lp_print_weighted_metrics("after phase-2 (SA)", &phase2_metrics, wt);
    }

    if (sa_apply_solution(&design, &problem, &phase2.solution, &dp_ss, &dp_ff, err,
                          sizeof(err)) != 0) {
        std::fprintf(stderr, "Apply failed: %s\n", err);
        sa_solution_free(&phase1);
        sa_solution_free(&phase2);
        lp_problem_free(&problem);
        pd_free_design(&design);
        return 1;
    }

    const double wall_elapsed = elapsed_sec(wall_t0);

    if (argc >= 3) {
        char struct_path[1024];
        mkdir(argv[2], 0755);

        if (pd_join_path(struct_path, sizeof(struct_path), argv[2],
                         "modified_clk_tree.structure") != 0) {
            std::fprintf(stderr, "Output path too long\n");
        } else if (pd_write_structure(&design, struct_path, err, sizeof(err)) != 0) {
            std::fprintf(stderr, "Write structure failed: %s\n", err);
        } else {
            std::printf("Wrote %s\n", struct_path);
        }

        if (ff_ori_clk_delay_write(&design, &problem, ori_ff_clk, argv[2], err, sizeof(err)) != 0)
            std::fprintf(stderr, "Write ff_ori_clk_delay failed: %s\n", err);

        if (seg_tree_write_outputs(&design, &problem, ori_ff_clk, argv[2], err, sizeof(err)) != 0)
            std::fprintf(stderr, "Write segment tree outputs failed: %s\n", err);

        LpPhaseResult phases[2];
        phases[0].phase_name = "phase-1 seg_tree_direct";
        phases[0].solver_name = phase1.solution.solver_name.c_str();
        phases[0].solver_status = phase1.solution.status;
        phases[0].elapsed_sec = phase1.elapsed_sec;
        phases[0].iterations = phase1.iterations;
        phases[0].metrics = phase1_metrics;

        phases[1].phase_name = "phase-2 batch_sa";
        phases[1].solver_name = phase2.solution.solver_name.c_str();
        phases[1].solver_status = phase2.solution.status;
        phases[1].elapsed_sec = phase2.elapsed_sec;
        phases[1].iterations = phase2.iterations;
        phases[1].metrics = phase2_metrics;

        if (lp_write_phased_result_txt(argv[2], testcase_dir, &ori, phases, 2, &wt,
                                       sa_params.no_improve_limit, wall_elapsed, err,
                                       sizeof(err)) != 0)
            std::fprintf(stderr, "Write result.txt failed: %s\n", err);
    }

    sa_solution_free(&phase1);
    sa_solution_free(&phase2);
    lp_problem_free(&problem);
    pd_free_design(&design);
    return 0;
}

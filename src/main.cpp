#include "insert_select.hpp"
#include "lp_buffer_dp.hpp"
#include "lp_mo_init.hpp"
#include "lp_score.hpp"
#include "lp_types.hpp"
#include "apply_solution.hpp"
#include "lp_eval.hpp"
#include "greedy_postlp.hpp"

#include <chrono>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <cmath>

namespace {

constexpr double kTailReserveSec = 15.0;

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

// Bundles the winning LpSolution plus the timing/iteration stats Phase 3 needs for the
// result.txt report - just a data carrier between Phase 2 (Greedy) and Phase 3 (Apply & Output).
struct SolverResult {
    LpSolution solution;
    double elapsed_sec = 0.0;
    double lp_init_sec = 0.0;
    int lp_init_ok = 0;
    int timed_out = 0;
    int use_second_best = 0;
    long long iterations = 0;
};

static void silence_stdio()
{
    if (!std::freopen("/dev/null", "w", stdout)) {
        /* keep default stdout if /dev/null is unavailable */
    }
    if (!std::freopen("/dev/null", "w", stderr)) {
        /* keep default stderr if /dev/null is unavailable */
    }
}

int main(int argc, char **argv)
{
    silence_stdio();

    PdDesign design{};
    PdDesign orig_design{}; // untouched copy of the input, kept for the final legality gate
    LpProblem problem;
    SolverResult solver_result{}; // carries the winning solution + stats through to Phase 3
    LpSolution lp_init{};
    LpMetrics ori{}, opt{};
    LpMetrics lp_init_metrics{};
    LpBufferChainDp dp_ss, dp_ff;
    char err[512];
    const char *testcase_dir;
    const char *output_path;
    const auto wall_t0 = std::chrono::steady_clock::now();

    if (argc < 3) {
        return 1;
    }

    testcase_dir = argv[1];
    output_path = argv[2];
    lp_problem_init(&problem);
    read_time_limit(&problem);
    if (problem.time_limit_sec <= 0.0)
        problem.time_limit_sec = 600.0;

    const double lp_init_limit = read_lp_init_limit();
    const double greedy_time_limit = read_greedy_time_limit();
    const auto wall_deadline =
        wall_t0 + std::chrono::duration_cast<SteadyClock::duration>(
                      std::chrono::duration<double>(
                          std::max(0.0, problem.time_limit_sec - kTailReserveSec)));


    if (pd_load_design(testcase_dir, &design, err, sizeof(err)) != 0) {
        return 1;
    }

    // Independent load (not a mutated copy of `design`) - this is the reference the final
    // legality gate compares against, the source of the *true* baseline ("_ori") metrics the
    // score is computed against, and the safe fallback output if the legality gate ever fails.
    if (pd_load_design(testcase_dir, &orig_design, err, sizeof(err)) != 0) {
        pd_free_design(&design);
        return 1;
    }

    if (lp_build_from_design(&problem, &design, err, sizeof(err)) != 0) {
        pd_free_design(&design);
        pd_free_design(&orig_design);
        return 1;
    }

    double dp_max_delay = 0.0;
    for (const LpBranch &br : problem.branches) {
        dp_max_delay = std::max(dp_max_delay, br.d_ss_max);
        dp_max_delay = std::max(dp_max_delay, br.d_ff_max);
    }
    dp_max_delay = std::min(LpBufferChainDp::kMaxDelay, dp_max_delay + 0.02);

    if (dp_ss.build(&design, LpBufferDpCorner::SS, dp_max_delay) != 0 ||
        dp_ff.build(&design, LpBufferDpCorner::FF, dp_max_delay) != 0) {
        pd_free_design(&design);
        pd_free_design(&orig_design);
        return 1;
    }

    // ---------------------------------------------------------
    // Phase 0: targeted buffer insertion
    // ---------------------------------------------------------
    // Decide where inserting a dedicated buffer can help by first solving the *resize-only*
    // exact relaxation (cheap - typically a few seconds even on the largest testcases) and
    // looking at which paths are *still* violating under that provably-optimal resize
    // solution. That is a far more precise signal than raw baseline violations: many baseline
    // violations get fully resolved by resize alone, and inserting there anyway would just be
    // a permanent, unrecoverable area tax (there is no "remove an unhelpful insertion" path).
    {
        const double pass1_budget =
            std::min(lp_init_limit, std::max(0.0, remaining_wall_sec(wall_deadline)));
        LpSolution lp_pass1;
        if (pass1_budget > 0.1 &&
            lp_solve_mo_init(&problem, &design, &lp_pass1, pass1_budget, err, sizeof(err)) == 0 &&
            !lp_pass1.d_ss.empty()) {
            LpEvalCtx pass1_ctx;
            if (lp_eval_build_ctx(&problem, &design, &pass1_ctx)) {
                lp_eval_state(&problem, &design, lp_pass1.d_ss, lp_pass1.d_ff, &dp_ss, &dp_ff,
                             &pass1_ctx);
                // pb->path_ids is the identity map 0..n_paths-1 (see lp_build_from_design), so
                // ctx path index p corresponds directly to design.paths[p].
                const std::size_t n_eval =
                    std::min(pass1_ctx.slack_ss.size(), static_cast<std::size_t>(design.n_paths));
                for (std::size_t p = 0; p < n_eval; p++) {
                    design.paths[p].slack_setup_ss = pass1_ctx.slack_ss[p];
                    design.paths[p].slack_hold_ff = pass1_ctx.slack_ff[p];
                }

                const int n_inserted = insert_decoupling_buffers(&design, err, sizeof(err));
                if (n_inserted < 0) {
                    (void)0;
                } else if (n_inserted > 0) {
                    // Give the resize search a "free" cell that only the buffers just inserted
                    // are allowed to pick (see LpBranch::allow_zero_cell) - resizing one all the
                    // way down to it and then physically removing it (Phase 3) is electrically
                    // identical to never having inserted it, so every insertion's area tax can
                    // be continuously reconsidered and undone instead of being a permanent,
                    // unrecoverable cost. Must exist before the lp_build_from_design rebuild
                    // below, since that's what computes each branch's bounds/eligibility.
                    if (pd_add_zero_cell(&design, nullptr, err, sizeof(err)) != 0) {
                        pd_free_design(&design);
                        pd_free_design(&orig_design);
                        return 1;
                    }

                    // Topology changed - rebuild everything that depends on it.
                    if (lp_build_from_design(&problem, &design, err, sizeof(err)) != 0) {
                        pd_free_design(&design);
                        pd_free_design(&orig_design);
                        return 1;
                    }
                    dp_max_delay = 0.0;
                    for (const LpBranch &br : problem.branches) {
                        dp_max_delay = std::max(dp_max_delay, br.d_ss_max);
                        dp_max_delay = std::max(dp_max_delay, br.d_ff_max);
                    }
                    dp_max_delay = std::min(LpBufferChainDp::kMaxDelay, dp_max_delay + 0.02);
                    if (dp_ss.build(&design, LpBufferDpCorner::SS, dp_max_delay) != 0 ||
                        dp_ff.build(&design, LpBufferDpCorner::FF, dp_max_delay) != 0) {
                        pd_free_design(&design);
                        pd_free_design(&orig_design);
                        return 1;
                    }
                }
            }
        } else {
            (void)0;
        }
    }

    // lp_build_from_design captured problem.*_ori from `design`'s *current* timing, which by
    // this point may already reflect the pre-inserted decoupling buffers - not the true
    // original baseline the contest scores against. Overwrite with the real thing, computed
    // from the untouched orig_design, before anything downstream reads these values.
    pd_annotate_clock(&orig_design);
    lp_compute_metrics(&orig_design, &ori);
    problem.wns_ss_ori = ori.wns_setup_ss;
    problem.tns_ss_ori = ori.tns_setup_ss;
    problem.wns_ff_ori = ori.wns_hold_ff;
    problem.tns_ff_ori = ori.tns_hold_ff;
    problem.area_ori = ori.area;
    ori.score = lp_compute_score(ori.wns_setup_ss, ori.tns_setup_ss, ori.wns_hold_ff, ori.tns_hold_ff,
                                 ori.area, problem.wns_ss_ori, problem.tns_ss_ori, problem.wns_ff_ori,
                                 problem.tns_ff_ori, problem.area_ori);
    (void)ori;

    std::vector<BranchDpOpts> opts;
    lp_eval_build_branch_opts(&problem, &design, &opts);

    // 保留 initial 作為 LP-init 沒有解時的 fallback，以及 Phase 2 的 dual-seed 起點
    LpSolution initial;
    lp_eval_init_from_design(&problem, &design, opts, &initial.d_ss, &initial.d_ff);

    const auto lp_t0 = std::chrono::steady_clock::now();

    // ---------------------------------------------------------
    // Phase 1: LP Init
    // ---------------------------------------------------------
    // Recomputed fresh (not the `lp_budget` from program start) - Phase 0's resize-only pass
    // and any insertion/rebuild it triggered may have already used up meaningful wall time,
    // and lp_solve_mo_init only knows about the budget it's given, not the global deadline.
    const double final_lp_budget =
        std::min(lp_init_limit, std::max(0.0, remaining_wall_sec(wall_deadline)));
    if (lp_solve_mo_init(&problem, &design, &lp_init, final_lp_budget, err, sizeof(err)) == 0 &&
        !lp_init.d_ss.empty()) {
        // NOTE: `initial` must keep holding the true original-design delays (not lp_init's) -
        // Phase 2 reuses it as a guaranteed-safe second seed for greedy.
        solver_result.lp_init_ok = 1;
        {
            LpEvalCtx lp_ctx;
            if (lp_eval_build_ctx(&problem, &design, &lp_ctx)) {
                lp_eval_state(&problem, &design, lp_init.d_ss, lp_init.d_ff, &dp_ss, &dp_ff, &lp_ctx);
                lp_init_metrics.wns_setup_ss = lp_ctx.wns_ss;
                lp_init_metrics.tns_setup_ss = lp_ctx.tns_ss;
                lp_init_metrics.wns_hold_ff = lp_ctx.wns_ff;
                lp_init_metrics.tns_hold_ff = lp_ctx.tns_ff;
                lp_init_metrics.area = lp_ctx.area;
                lp_init_metrics.score = lp_ctx.score;
            }
        }
    } else {
        solver_result.lp_init_ok = 0;
    }
    solver_result.lp_init_sec = std::chrono::duration<double>(std::chrono::steady_clock::now() - lp_t0).count();
// 強制將 LP 初始解轉換為真實圖形，並請出真實裁判計分
    if (solver_result.lp_init_ok) {
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
                if (!br.allow_zero_cell && pd_cell_is_zero(&design.cells[ci])) continue;
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
    if (solver_result.lp_init_ok) {
        const double greedy_budget =
            std::min(greedy_time_limit, std::max(0.0, remaining_wall_sec(wall_deadline)));

        // Greedy's single-buffer hill-climbing is path-dependent: a better LP-init start does
        // not always reach a better local optimum than starting from the plain original design
        // (observed empirically on some testcases). We always run two rounds (LP-init seed,
        // original-design seed) and keep whichever final result actually scores higher - but
        // whichever of the two *currently* scores higher goes first and gets the larger budget,
        // so the two rounds always explore two genuinely different starting points. (Previously
        // round 1 always used the LP-init seed unconditionally, and greedy_post_lp silently
        // reverted it to the original design internally whenever LP init scored worse than
        // baseline - making round 2's baseline-seeded run an exact, wasted duplicate of round 1
        // in that case, since both then started from the same snapshot with the same fixed RNG
        // seed.)
        LpSolution baseline_seed = initial;
        LpMetrics baseline_metrics{};
        baseline_metrics.wns_setup_ss = problem.wns_ss_ori;
        baseline_metrics.tns_setup_ss = problem.tns_ss_ori;
        baseline_metrics.wns_hold_ff = problem.wns_ff_ori;
        baseline_metrics.tns_hold_ff = problem.tns_ff_ori;
        baseline_metrics.area = problem.area_ori;
        baseline_metrics.score = 0.0;

        LpSolution *first_seed = &lp_init;
        LpMetrics *first_metrics = &lp_init_metrics;
        const char *first_name = "Greedy_Best_Impr";
        LpSolution *second_seed = &baseline_seed;
        LpMetrics *second_seed_metrics = &baseline_metrics;
        const char *second_name = "Greedy_Best_Impr_OrigSeed";
        if (lp_init_metrics.score < 0.0) {
            std::swap(first_seed, second_seed);
            std::swap(first_metrics, second_seed_metrics);
            std::swap(first_name, second_name);
        }

        if (greedy_budget > 0.1 &&
            greedy_post_lp(nullptr, testcase_dir, &problem, &design, &dp_ss, &dp_ff, first_seed,
                           first_metrics, greedy_budget, wall_deadline, err,
                           sizeof(err)) == 0) {
            LpMetrics best_metrics{};
            lp_compute_metrics(&design, &best_metrics);
            best_metrics.score = lp_compute_score(
                best_metrics.wns_setup_ss, best_metrics.tns_setup_ss, best_metrics.wns_hold_ff,
                best_metrics.tns_hold_ff, best_metrics.area, problem.wns_ss_ori, problem.tns_ss_ori,
                problem.wns_ff_ori, problem.tns_ff_ori, problem.area_ori);
            LpSolution best_solution = *first_seed;
            const char *best_name = first_name;

            const double remaining_after = remaining_wall_sec(wall_deadline);
            if (remaining_after > 5.0) {
                if (greedy_post_lp(nullptr, testcase_dir, &problem, &design, &dp_ss, &dp_ff,
                                   second_seed, second_seed_metrics, remaining_after,
                                   wall_deadline, err, sizeof(err)) == 0) {
                    LpMetrics second_round_metrics{};
                    lp_compute_metrics(&design, &second_round_metrics);
                    second_round_metrics.score = lp_compute_score(
                        second_round_metrics.wns_setup_ss, second_round_metrics.tns_setup_ss,
                        second_round_metrics.wns_hold_ff, second_round_metrics.tns_hold_ff,
                        second_round_metrics.area, problem.wns_ss_ori, problem.tns_ss_ori,
                        problem.wns_ff_ori, problem.tns_ff_ori, problem.area_ori);
                    if (second_round_metrics.score > best_metrics.score) {
                        best_metrics = second_round_metrics;
                        best_solution = *second_seed;
                        best_name = second_name;
                    }
                }
            }

            // 將較佳的 Greedy 結果裝進最終要輸出的結構裡
            solver_result.solution.d_ss = best_solution.d_ss;
            solver_result.solution.d_ff = best_solution.d_ff;
            solver_result.solution.status = 1;
            solver_result.solution.solver_name = best_name;
        } else if (greedy_budget <= 0.1) {
            solver_result.solution = lp_init;
        } else {
            solver_result.solution = lp_init;
        }
    } else {
        solver_result.solution = initial;
    }
    solver_result.elapsed_sec = std::chrono::duration<double>(std::chrono::steady_clock::now() - lp_t0).count();

    // ---------------------------------------------------------
    // Phase 3: Apply & Output
    // ---------------------------------------------------------
    if (apply_solution(&design, &problem, &solver_result.solution, &dp_ss, &dp_ff, err,
                       sizeof(err)) != 0) {
        lp_problem_free(&problem);
        pd_free_design(&design);
        pd_free_design(&orig_design);
        return 1;
    }

    // Any NEW_BUF_* the search shrank all the way down to the synthetic zero-area/zero-delay
    // cell was already contributing exactly zero to both timing and area - physically remove
    // it now so the output never references a cell the real library doesn't contain. Only ever
    // targets NEW_BUF_* names (never an existing component), matching pd_remove_buffer's
    // contract; if a zero-cell assignment somehow ended up elsewhere, leave it for the legality
    // gate below to catch instead of silently deleting something it shouldn't.
    {
        int n_removed = 0;
        for (int i = 0; i < design.n_nodes; i++) {
            PdNode &n = design.nodes[i];
            if (n.kind != PD_NODE_BUF || n.cell_idx < 0)
                continue;
            if (!pd_cell_is_zero(&design.cells[n.cell_idx]))
                continue;
            if (!pd_is_new_buf_name(n.name))
                continue;
            char rm_err[256];
            if (pd_remove_buffer(&design, i, rm_err, sizeof(rm_err)) != 0) {
                continue;
            }
            n_removed++;
        }
        if (n_removed > 0) {
            pd_annotate_clock(&design);
            pd_compute_timing(&design);
        }
    }

    lp_compute_metrics(&design, &opt);
    opt.score = lp_compute_score(opt.wns_setup_ss, opt.tns_setup_ss, opt.wns_hold_ff, opt.tns_hold_ff,
                                 opt.area, problem.wns_ss_ori, problem.tns_ss_ori, problem.wns_ff_ori,
                                 problem.tns_ff_ori, problem.area_ori);
    (void)opt;

    // Final safety gate: the contest disqualifies (score 0) any testcase whose output violates
    // the structural rules (existing components preserved & in the same relative order, no
    // dangling pins, fanout within each cell's limit, unique names, new buffers named
    // NEW_BUF_X). If our own search ever produced an illegal result despite our best efforts,
    // fall back to re-emitting the untouched original design - a guaranteed-legal, zero-
    // improvement result is far better than risking disqualification.
    const PdDesign *design_to_write = &design;
    if (pd_check_legality(&orig_design, &design, err, sizeof(err)) != 0) {
        design_to_write = &orig_design;
    }

    if (pd_write_structure(design_to_write, output_path, err, sizeof(err)) != 0) {
        lp_problem_free(&problem);
        pd_free_design(&design);
        pd_free_design(&orig_design);
        return 1;
    }

    lp_problem_free(&problem);
    pd_free_design(&design);
    pd_free_design(&orig_design);
    return 0;
}
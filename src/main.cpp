#include "lp_apply.hpp"
#include "lp_score.hpp"
#include "lp_solve.hpp"
#include "lp_types.hpp"

#include <cstdlib>
#include <cstdio>
#include <cstring>

static void read_time_limit(LpProblem *pb)
{
    const char *env = std::getenv("LP_TIME_LIMIT");
    if (env && env[0]) {
        const double t = std::atof(env);
        if (t > 0.1)
            pb->time_limit_sec = t;
    }
}

int main(int argc, char **argv)
{
    PdDesign design{};
    LpProblem problem;
    LpSolution solution;
    LpMetrics ori{}, opt{};
    char err[512];
    const char *testcase_dir;

    if (argc < 2) {
        std::fprintf(stderr, "Usage: %s <testcase_dir> [result_dir]\n", argv[0]);
        std::fprintf(stderr, "  result_dir: writes result.txt and modified_clk_tree.structure\n");
        return 1;
    }

    testcase_dir = argv[1];
    lp_problem_init(&problem);
    read_time_limit(&problem);

    std::printf("=== lp_solver ===\n");
    std::printf("Input folder: %s\n", testcase_dir);
    std::printf("Time limit  : %.1f sec\n", problem.time_limit_sec);

    if (pd_load_design(testcase_dir, &design, err, sizeof(err)) != 0) {
        std::fprintf(stderr, "Load failed: %s\n", err);
        return 1;
    }

    if (lp_build_from_design(&problem, &design, err, sizeof(err)) != 0) {
        std::fprintf(stderr, "LP build failed: %s\n", err);
        pd_free_design(&design);
        return 1;
    }

    lp_compute_metrics(&design, &ori);
    ori.score = lp_compute_score(ori.wns_setup_ss, ori.tns_setup_ss, ori.wns_hold_ff, ori.tns_hold_ff,
                                 ori.area, problem.wns_ss_ori, problem.tns_ss_ori, problem.wns_ff_ori,
                                 problem.tns_ff_ori, problem.area_ori);
    lp_print_metrics("baseline (ori)", &ori);
    std::printf("(ori reference) WNS_SS=%.6f TNS_SS=%.6f WNS_FF=%.6f TNS_FF=%.6f Area=%.6f\n",
                problem.wns_ss_ori, problem.tns_ss_ori, problem.wns_ff_ori, problem.tns_ff_ori,
                problem.area_ori);

    std::printf("Branches: %zu  FF: %zu  Paths: %zu\n", problem.branches.size(),
                problem.ff_node_ids.size(), problem.path_ids.size());

    if (lp_solve(&problem, &design, &solution, err, sizeof(err)) != 0) {
        std::fprintf(stderr, "Solve failed: %s\n", err);
        lp_problem_free(&problem);
        pd_free_design(&design);
        return 1;
    }

    std::printf("Solver: %s (status=%d)\n", solution.solver_name.c_str(), solution.status);

    if (lp_apply_solution(&design, &problem, &solution, err, sizeof(err)) != 0) {
        std::fprintf(stderr, "Apply failed: %s\n", err);
        lp_solution_free(&solution);
        lp_problem_free(&problem);
        pd_free_design(&design);
        return 1;
    }

    lp_compute_metrics(&design, &opt);
    opt.score = lp_compute_score(opt.wns_setup_ss, opt.tns_setup_ss, opt.wns_hold_ff, opt.tns_hold_ff,
                                 opt.area, problem.wns_ss_ori, problem.tns_ss_ori, problem.wns_ff_ori,
                                 problem.tns_ff_ori, problem.area_ori);
    lp_print_metrics("after LP", &opt);

    if (argc >= 3) {
        char struct_path[1024];

        if (lp_write_result_txt(argv[2], testcase_dir, &ori, &opt, solution.solver_name.c_str(),
                                solution.status, problem.time_limit_sec, err, sizeof(err)) != 0) {
            std::fprintf(stderr, "Write result.txt failed: %s\n", err);
        } else {
            char result_txt[1024];
            if (pd_join_path(result_txt, sizeof(result_txt), argv[2], "result.txt") == 0)
                std::printf("Wrote %s\n", result_txt);
        }

        if (pd_join_path(struct_path, sizeof(struct_path), argv[2],
                         "modified_clk_tree.structure") != 0) {
            std::fprintf(stderr, "Output path too long\n");
        } else if (pd_write_structure(&design, struct_path, err, sizeof(err)) != 0) {
            std::fprintf(stderr, "Write structure failed: %s\n", err);
        } else {
            std::printf("Wrote %s\n", struct_path);
        }
    }

    lp_solution_free(&solution);
    lp_problem_free(&problem);
    pd_free_design(&design);
    return 0;
}

#include "lp_print_input.hpp"
#include "lp_types.hpp"

#include <cstdio>
#include <cstring>

int main(int argc, char **argv)
{
    const char *testcase_dir = (argc >= 2) ? argv[1] : "testcase/testcase1";
    const char *out_dir = (argc >= 3) ? argv[2] : "result/print_lp_1";

    PdDesign design{};
    LpProblem problem;
    char err[512];

    std::printf("print_lp_input: testcase=%s out=%s\n", testcase_dir, out_dir);

    if (pd_load_design(testcase_dir, &design, err, sizeof(err)) != 0) {
        std::fprintf(stderr, "Load failed: %s\n", err);
        return 1;
    }

    lp_problem_init(&problem);
    if (lp_build_from_design(&problem, &design, err, sizeof(err)) != 0) {
        std::fprintf(stderr, "Build failed: %s\n", err);
        pd_free_design(&design);
        return 1;
    }

    if (lp_write_mo_input_dump(&problem, &design, out_dir, err, sizeof(err)) != 0) {
        std::fprintf(stderr, "Dump failed: %s\n", err);
        lp_problem_free(&problem);
        pd_free_design(&design);
        return 1;
    }

    std::printf("Wrote LP input dump to %s/\n", out_dir);
    lp_problem_free(&problem);
    pd_free_design(&design);
    return 0;
}

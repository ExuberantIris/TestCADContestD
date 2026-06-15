#include "lp_score.hpp"
#include "pd_c_api.hpp"

#include <cstdio>
#include <cstring>

static int load_and_compute(const char *testcase_dir, const char *structure_path, LpMetrics *m,
                            char *err, std::size_t err_sz)
{
    PdDesign design{};

    if (pd_load_design_with_structure(testcase_dir, structure_path, &design, err, err_sz) != 0)
        return -1;

    pd_annotate_clock(&design);
    lp_compute_metrics(&design, m);
    pd_free_design(&design);
    return 0;
}

int main(int argc, char **argv)
{
    if (argc < 3) {
        std::fprintf(stderr, "Usage: %s <testcase_dir> <modified_clk_tree.structure>\n", argv[0]);
        return 1;
    }

    const char *testcase_dir = argv[1];
    const char *structure_path = argv[2];
    char ori_struct_path[1024];
    char err[512];
    LpMetrics ori{};
    LpMetrics opt{};

    if (pd_join_path(ori_struct_path, sizeof(ori_struct_path), testcase_dir,
                     "clk_tree.structure") != 0) {
        std::fprintf(stderr, "Original structure path too long\n");
        return 1;
    }

    if (load_and_compute(testcase_dir, ori_struct_path, &ori, err, sizeof(err)) != 0) {
        std::fprintf(stderr, "Load original failed: %s\n", err);
        return 1;
    }

    if (load_and_compute(testcase_dir, structure_path, &opt, err, sizeof(err)) != 0) {
        std::fprintf(stderr, "Load modified failed: %s\n", err);
        return 1;
    }

    opt.score = lp_compute_timing_score(opt.wns_setup_ss, opt.tns_setup_ss, opt.wns_hold_ff,
                                        opt.tns_hold_ff, ori.wns_setup_ss, ori.tns_setup_ss,
                                        ori.wns_hold_ff, ori.tns_hold_ff);

    std::printf("testcase_dir: %s\n", testcase_dir);
    std::printf("original_structure: %s\n", ori_struct_path);
    std::printf("modified_structure: %s\n", structure_path);
    std::printf("\n");

    lp_print_metrics("baseline (original clk tree)", &ori);
    std::printf("\n");
    lp_print_metrics("after optimize", &opt);

    return 0;
}

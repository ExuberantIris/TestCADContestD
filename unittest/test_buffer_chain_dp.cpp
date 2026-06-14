#include "lp_buffer_dp.hpp"
#include "lp_types.hpp"

#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>

static std::string resolve_testcase_dir(int argc, char **argv)
{
    if (argc >= 2 && argv[1][0] != '\0')
        return argv[1];

    const char *candidates[] = {"testcase/testcase1", "../testcase/testcase1",
                                "ProblemD/testcase/testcase1"};
    for (const char *p : candidates) {
        char path[512];
        std::snprintf(path, sizeof(path), "%s/buf.lib", p);
        FILE *fp = std::fopen(path, "r");
        if (fp) {
            std::fclose(fp);
            return p;
        }
    }
    return "testcase/testcase1";
}

int main(int argc, char **argv)
{
    const std::string testcase_dir = resolve_testcase_dir(argc, argv);
    PdDesign design{};
    char err[512];

    std::cout << "=== unittest: buffer chain DP (testcase1) ===\n";
    std::cout << "testcase_dir: " << testcase_dir << "\n\n";

    if (pd_load_design(testcase_dir.c_str(), &design, err, sizeof(err)) != 0) {
        std::cerr << "Load failed: " << err << "\n";
        return 1;
    }

    LpBufferChainDp dp_ss;
    LpBufferChainDp dp_ff;
    if (dp_ss.build(&design, LpBufferDpCorner::SS) != 0 ||
        dp_ff.build(&design, LpBufferDpCorner::FF) != 0) {
        std::cerr << "DP build failed\n";
        pd_free_design(&design);
        return 2;
    }

    dp_ss.print_table(std::cout, LpBufferDpCorner::SS);
    dp_ff.print_table(std::cout, LpBufferDpCorner::FF);

    std::cout << "--- sample lookup (SS, fanout=3, delay=0.08) ---\n";
    const LpBufferChainEntry &e = dp_ss.lookup(3, 0.08);
    if (e.reachable) {
        std::cout << "  area=" << e.area << " delay=" << e.delay << " chain: ";
        for (std::size_t i = 0; i < e.cell_indices.size(); i++) {
            if (i > 0)
                std::cout << " -> ";
            std::cout << design.cells[e.cell_indices[i]].name;
        }
        std::cout << "\n";
    } else {
        std::cout << "  (unreachable at this delay)\n";
    }

    std::cout << "\n=== run sa_solver on same testcase ===\n";
    std::cout.flush();

    const std::string result_dir = "result/testcase1_unittest";
    mkdir(result_dir.c_str(), 0755);
    std::string cmd = "./sa_solver \"" + testcase_dir + "\" \"" + result_dir + "\"";
    const char *limit = std::getenv("SA_TIME_LIMIT");
    if (!limit || !limit[0])
        limit = std::getenv("LP_TIME_LIMIT");
    if (!limit || !limit[0])
        cmd = "SA_TIME_LIMIT=60 " + cmd;
    else
        cmd = std::string("SA_TIME_LIMIT=") + limit + " " + cmd;

    std::cout << "command: " << cmd << "\n\n";
    const int rc = std::system(cmd.c_str());
    if (rc != 0) {
        std::cerr << "sa_solver exited with status " << rc << "\n";
        pd_free_design(&design);
        return 3;
    }

    pd_free_design(&design);
    std::cout << "\nPASS\n";
    return 0;
}

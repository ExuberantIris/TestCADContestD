#include "pd_c_api.hpp"
#include "insert_select.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <set>
#include <string>
#include <sys/stat.h>
#include <vector>

namespace {

std::string resolve_testcase_dir(int argc, char **argv)
{
    if (argc >= 2 && argv[1][0] != '\0')
        return argv[1];

    const char *candidates[] = {"testcase/testcase2", "../testcase/testcase2",
                                "ProblemD/testcase/testcase2"};
    for (const char *p : candidates) {
        char path[512];
        std::snprintf(path, sizeof(path), "%s/buf.lib", p);
        FILE *fp = std::fopen(path, "r");
        if (fp) {
            std::fclose(fp);
            return p;
        }
    }
    return "testcase/testcase2";
}

int g_pass = 0;
int g_fail = 0;

void check(bool cond, const std::string &label)
{
    if (cond) {
        std::cout << "  [PASS] " << label << "\n";
        g_pass++;
    } else {
        std::cout << "  [FAIL] " << label << "\n";
        g_fail++;
    }
}

/** First FF whose direct parent already drives more than one child - just a convenient,
 *  real-data spot check; pd_find_branch_point no longer treats this any differently from any
 *  other FF with a buffer parent. */
int find_ff_with_branching_direct_parent(const PdDesign &d)
{
    for (int i = 0; i < d.n_nodes; i++) {
        if (d.nodes[i].kind != PD_NODE_FF)
            continue;
        const int p = d.nodes[i].parent;
        if (p >= 0 && d.nodes[p].kind == PD_NODE_BUF && d.nodes[p].nchildren > 1)
            return i;
    }
    return -1;
}

} // namespace

int main(int argc, char **argv)
{
    const std::string testcase_dir = resolve_testcase_dir(argc, argv);
    char err[512];

    std::cout << "=== unittest: pd_find_branch_point / insert_decoupling_buffers "
                 "(direct-parent-only) ===\n";
    std::cout << "testcase_dir: " << testcase_dir << "\n\n";

    PdDesign d{};
    if (pd_load_design(testcase_dir.c_str(), &d, err, sizeof(err)) != 0) {
        std::cerr << "Load failed: " << err << "\n";
        return 1;
    }
    pd_annotate_clock(&d);
    pd_compute_timing(&d);

    // --- Case 1: direct parent branches - returns (parent, ff), and is deterministic ----------
    {
        const int ff = find_ff_with_branching_direct_parent(d);
        if (ff < 0) {
            std::cout << "  [SKIP] no FF with a branching direct parent found\n";
        } else {
            int ancestor = -1, child = -1;
            const bool ok = pd_find_branch_point(&d, ff, &ancestor, &child);
            check(ok, "branch point found for an FF whose direct parent branches");
            check(ancestor == d.nodes[ff].parent, "...and it is exactly the direct parent");
            check(child == ff, "...with child == the FF itself");

            // Same ff_id must always resolve to the same (ancestor, child) edge - this is the
            // determinism insert_decoupling_buffers' std::set-based dedup relies on, regardless
            // of how many violating paths re-discover the same FF.
            int ancestor2 = -1, child2 = -1;
            const bool ok2 = pd_find_branch_point(&d, ff, &ancestor2, &child2);
            check(ok2 && ancestor2 == ancestor && child2 == child,
                 "...repeated calls for the same ff_id return the identical edge (dedup-safe)");
        }
    }

    // --- Case 2: direct parent is single-child, deep in an unbranched chain - must NOT walk
    // further up; returns the direct parent unchanged, since splicing a buffer anywhere along an
    // exclusive chain has an identical timing effect (no fork ever exists in this hand-built
    // tree, so the *old* walk-to-fork logic would have returned false here). -------------------
    {
        PdNode nodes[4] = {};
        int root_children[1] = {1};
        int buf1_children[1] = {2};
        int buf2_children[1] = {3};

        nodes[0].id = 0;
        nodes[0].kind = PD_NODE_ROOT;
        nodes[0].parent = -1;
        nodes[0].children = root_children;
        nodes[0].nchildren = 1;

        nodes[1].id = 1;
        nodes[1].kind = PD_NODE_BUF;
        nodes[1].parent = 0;
        nodes[1].children = buf1_children;
        nodes[1].nchildren = 1;

        nodes[2].id = 2;
        nodes[2].kind = PD_NODE_BUF;
        nodes[2].parent = 1;
        nodes[2].children = buf2_children;
        nodes[2].nchildren = 1;

        nodes[3].id = 3;
        nodes[3].kind = PD_NODE_FF;
        nodes[3].parent = 2;
        nodes[3].nchildren = 0;

        PdDesign chain{};
        chain.nodes = nodes;
        chain.n_nodes = 4;

        int ancestor = -1, child = -1;
        const bool ok = pd_find_branch_point(&chain, 3, &ancestor, &child);
        check(ok, "branch point found even though no fork exists anywhere in the chain");
        check(ancestor == 2, "...and it is exactly the FF's direct parent, not walked further up");
        check(child == 3, "...with child == the FF itself");
    }

    // --- Case 3: FF's direct parent is the root (no buffer in between) - must report no branch
    // point, matching pd_insert_buffer_on_child's requirement that the parent be a real buffer.
    {
        PdNode nodes[2] = {};
        int root_children[1] = {1};

        nodes[0].id = 0;
        nodes[0].kind = PD_NODE_ROOT;
        nodes[0].parent = -1;
        nodes[0].children = root_children;
        nodes[0].nchildren = 1;

        nodes[1].id = 1;
        nodes[1].kind = PD_NODE_FF;
        nodes[1].parent = 0;
        nodes[1].nchildren = 0;

        PdDesign rootonly{};
        rootonly.nodes = nodes;
        rootonly.n_nodes = 2;

        int ancestor = -1, child = -1;
        const bool ok = pd_find_branch_point(&rootonly, 1, &ancestor, &child);
        check(!ok, "an FF driven directly by the root correctly reports no branch point");
    }

    // --- End-to-end: run insert_decoupling_buffers and sanity-check the result ----------------
    {
        const int n_inserted = insert_decoupling_buffers(&d, err, sizeof(err));
        check(n_inserted >= 0, "insert_decoupling_buffers succeeds");
        std::printf("insert_decoupling_buffers: %d buffer(s) inserted (direct-parent-only)\n",
                   n_inserted);

        std::set<std::string> names;
        bool unique_names = true;
        for (int i = 0; i < d.n_nodes; i++)
            if (!names.insert(d.nodes[i].name).second)
                unique_names = false;
        check(unique_names, "all node names unique after insertion (no duplicate-edge insert)");
    }

    PdDesign orig{};
    if (pd_load_design(testcase_dir.c_str(), &orig, err, sizeof(err)) == 0) {
        char e[512];
        check(pd_check_legality(&orig, &d, e, sizeof(e)) == 0,
             "insertion result passes pd_check_legality");
        pd_free_design(&orig);
    }

    pd_free_design(&d);

    std::cout << "\n" << g_pass << " passed, " << g_fail << " failed\n";
    return g_fail == 0 ? 0 : 1;
}

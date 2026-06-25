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

/** First FF whose direct parent already drives more than one child - the case the *old*
 *  direct-parent-only logic already handled; pd_find_branch_point must still return exactly
 *  this same edge for it (old behavior is the n=0-extra-hops special case of the new walk). */
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

/** First FF whose direct parent is a *single-child* buffer (the case the old logic missed
 *  entirely), walking up further to require at least one more single-child hop before any
 *  branch point, so the test actually exercises "walk past more than one level". */
int find_ff_needing_multi_hop_walk(const PdDesign &d, int min_single_child_hops)
{
    for (int i = 0; i < d.n_nodes; i++) {
        if (d.nodes[i].kind != PD_NODE_FF)
            continue;
        int hops = 0;
        int cur = d.nodes[i].parent;
        while (cur >= 0 && d.nodes[cur].kind == PD_NODE_BUF && d.nodes[cur].nchildren == 1) {
            hops++;
            cur = d.nodes[cur].parent;
        }
        if (hops >= min_single_child_hops && cur >= 0 && d.nodes[cur].kind == PD_NODE_BUF &&
            d.nodes[cur].nchildren > 1)
            return i;
    }
    return -1;
}

} // namespace

int main(int argc, char **argv)
{
    const std::string testcase_dir = resolve_testcase_dir(argc, argv);
    char err[512];

    std::cout << "=== unittest: pd_find_branch_point / insert_decoupling_buffers (widened) ===\n";
    std::cout << "testcase_dir: " << testcase_dir << "\n\n";

    PdDesign d{};
    if (pd_load_design(testcase_dir.c_str(), &d, err, sizeof(err)) != 0) {
        std::cerr << "Load failed: " << err << "\n";
        return 1;
    }
    pd_annotate_clock(&d);
    pd_compute_timing(&d);

    // --- Case 1: direct parent already branches - must match the old behavior exactly -------
    {
        const int ff = find_ff_with_branching_direct_parent(d);
        if (ff < 0) {
            std::cout << "  [SKIP] no FF with a branching direct parent found\n";
        } else {
            int ancestor = -1, child = -1;
            const bool ok = pd_find_branch_point(&d, ff, &ancestor, &child);
            check(ok, "branch point found for an FF whose direct parent already branches");
            check(ancestor == d.nodes[ff].parent,
                 "...and it is exactly the direct parent (0-extra-hop case unchanged)");
            check(child == ff, "...with child == the FF itself, matching the old (parent, ff) edge");
        }
    }

    // --- Case 2: direct parent is single-child - must walk past it to the real branch point -
    {
        const int ff = find_ff_needing_multi_hop_walk(d, 1);
        if (ff < 0) {
            std::cout << "  [SKIP] no FF requiring a multi-hop walk found in this testcase\n";
        } else {
            int ancestor = -1, child = -1;
            const bool ok = pd_find_branch_point(&d, ff, &ancestor, &child);
            check(ok, "branch point found by walking past single-child ancestors");
            check(ancestor != d.nodes[ff].parent,
                 "...and it is NOT the direct parent (this is the case the old logic missed)");
            check(d.nodes[ancestor].kind == PD_NODE_BUF && d.nodes[ancestor].nchildren > 1,
                 "...the returned ancestor genuinely branches");
            // child must be the one of ancestor's direct children whose subtree contains ff.
            bool child_is_direct_child_of_ancestor = false;
            for (int k = 0; k < d.nodes[ancestor].nchildren; k++)
                if (d.nodes[ancestor].children[k] == child)
                    child_is_direct_child_of_ancestor = true;
            check(child_is_direct_child_of_ancestor,
                 "...child is one of the ancestor's direct children");
        }
    }

    // --- Case 3: no branch point exists below the root - hand-built unbranched chain
    // (root -> single-child buf -> single-child buf -> FF). Also covers "the branch point
    // would be the root itself": the root is PD_NODE_ROOT, never PD_NODE_BUF, so the walk's
    // loop condition must stop before ever considering it a valid insertion point (matching
    // pd_insert_buffer_on_child's own requirement that the parent be a real buffer). ----------
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
        check(!ok, "an entirely unbranched chain up to the root correctly reports no branch point");
    }

    // --- End-to-end: run the widened insert_decoupling_buffers and sanity-check the result --
    {
        const int n_inserted = insert_decoupling_buffers(&d, err, sizeof(err));
        check(n_inserted >= 0, "insert_decoupling_buffers succeeds");
        std::printf("insert_decoupling_buffers: %d buffer(s) inserted (widened branch-point search)\n",
                   n_inserted);

        std::set<std::string> names;
        bool unique_names = true;
        for (int i = 0; i < d.n_nodes; i++)
            if (!names.insert(d.nodes[i].name).second)
                unique_names = false;
        check(unique_names, "all node names unique after widened insertion");
    }

    PdDesign orig{};
    if (pd_load_design(testcase_dir.c_str(), &orig, err, sizeof(err)) == 0) {
        char e[512];
        check(pd_check_legality(&orig, &d, e, sizeof(e)) == 0,
             "widened insertion result passes pd_check_legality");
        pd_free_design(&orig);
    }

    pd_free_design(&d);

    std::cout << "\n" << g_pass << " passed, " << g_fail << " failed\n";
    return g_fail == 0 ? 0 : 1;
}

#include "pd_c_api.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>

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

int find_buffer_with_children(const PdDesign &d, int min_children)
{
    for (int i = 0; i < d.n_nodes; i++)
        if (d.nodes[i].kind == PD_NODE_BUF && d.nodes[i].nchildren >= min_children)
            return i;
    return -1;
}

int g_pass = 0;
int g_fail = 0;

void expect(bool cond, const std::string &label)
{
    if (cond) {
        std::cout << "  [PASS] " << label << "\n";
        g_pass++;
    } else {
        std::cout << "  [FAIL] " << label << "\n";
        g_fail++;
    }
}

/** PdDesign holds raw owning pointers (nodes/paths, and each node's children array), so a
 *  struct copy would alias the original instead of giving an independent design to corrupt.
 *  Each corruption test instead reloads + re-splits from scratch via this helper. */
struct SplitFixture {
    PdDesign design{};
    int target = -1;
    int new_node_id = -1;
    bool ok = false;
};

SplitFixture build_split_fixture(const std::string &testcase_dir)
{
    SplitFixture f;
    char err[512];

    if (pd_load_design(testcase_dir.c_str(), &f.design, err, sizeof(err)) != 0) {
        std::cerr << "Load failed: " << err << "\n";
        return f;
    }
    f.target = find_buffer_with_children(f.design, 2);
    if (f.target < 0) {
        std::cerr << "No splittable buffer found\n";
        return f;
    }
    int new_cell_idx = -1;
    for (int i = 0; i < f.design.n_cells; i++) {
        if (f.design.cells[i].max_fanout >= f.design.nodes[f.target].nchildren) {
            new_cell_idx = i;
            break;
        }
    }
    if (pd_split_buffer(&f.design, f.target, new_cell_idx, &f.new_node_id, err, sizeof(err)) != 0) {
        std::cerr << "pd_split_buffer failed: " << err << "\n";
        return f;
    }
    f.ok = true;
    return f;
}

/** Expects pd_check_legality to reject `mod`, with the error message containing `needle`. */
void expect_rejected(const PdDesign &orig, const PdDesign &mod, const char *needle,
                    const std::string &label)
{
    char err[512] = {0};
    const int rc = pd_check_legality(&orig, &mod, err, sizeof(err));
    const bool got_expected_error = (rc != 0) && (std::strstr(err, needle) != nullptr);
    if (!got_expected_error)
        std::cout << "    (rc=" << rc << " err=\"" << err << "\")\n";
    expect(got_expected_error, label);
}

} // namespace

int main(int argc, char **argv)
{
    const std::string testcase_dir = resolve_testcase_dir(argc, argv);

    std::cout << "=== unittest: pd_check_legality ===\n";
    std::cout << "testcase_dir: " << testcase_dir << "\n\n";

    PdDesign orig{};
    char err[512];
    if (pd_load_design(testcase_dir.c_str(), &orig, err, sizeof(err)) != 0) {
        std::cerr << "Load failed: " << err << "\n";
        return 1;
    }

    // --- Happy path -------------------------------------------------------------------------
    {
        char e[512];
        expect(pd_check_legality(&orig, &orig, e, sizeof(e)) == 0,
              "original design passes legality check against itself");
    }

    SplitFixture good = build_split_fixture(testcase_dir);
    if (!good.ok) {
        pd_free_design(&orig);
        return 2;
    }
    {
        char e[512];
        expect(pd_check_legality(&orig, &good.design, e, sizeof(e)) == 0,
              "legally split design passes legality check");
    }
    pd_free_design(&good.design);

    // --- Deliberately corrupted variants: each gets its own fresh fixture, then one targeted
    // corruption, then must be rejected with the expected reason. ---------------------------
    {
        SplitFixture f = build_split_fixture(testcase_dir);
        std::strncpy(f.design.nodes[f.target].name, "RENAMED_BUF", PD_MAX_NAME - 1);
        expect_rejected(orig, f.design, "deleted or renamed",
                       "renaming an existing component is rejected");
        pd_free_design(&f.design);
    }
    {
        SplitFixture f = build_split_fixture(testcase_dir);
        std::strncpy(f.design.nodes[f.new_node_id].name, f.design.nodes[f.target].name,
                    PD_MAX_NAME - 1);
        expect_rejected(orig, f.design, "duplicate", "duplicate component name is rejected");
        pd_free_design(&f.design);
    }
    {
        // Shrink an assigned cell's max_fanout field below the buffer's actual (real, valid)
        // fanout. This tests the exact same nchildren > cell.max_fanout comparison as a real
        // violation would, without depending on the library happening to contain a cell small
        // enough relative to some buffer's fanout (e.g. testcase0's library is uniform,
        // max_fanout=5 for every cell, so no real buffer can ever exceed it).
        PdDesign d{};
        char e[512];
        if (pd_load_design(testcase_dir.c_str(), &d, e, sizeof(e)) != 0) {
            std::cerr << "Load failed: " << e << "\n";
        } else {
            int big = -1;
            for (int i = 0; i < d.n_nodes; i++) {
                if (d.nodes[i].kind == PD_NODE_BUF && d.nodes[i].nchildren >= 1) {
                    big = i;
                    break;
                }
            }
            PdCell &c = d.cells[d.nodes[big].cell_idx];
            const int saved_max_fanout = c.max_fanout;
            c.max_fanout = d.nodes[big].nchildren - 1;
            expect_rejected(d, d, "max_fanout", "exceeding a cell's max_fanout is rejected");
            c.max_fanout = saved_max_fanout;
            pd_free_design(&d);
        }
    }
    {
        SplitFixture f = build_split_fixture(testcase_dir);
        // Give an FF a child - FF must be a sink.
        int ff = -1;
        for (int i = 0; i < f.design.n_nodes; i++)
            if (f.design.nodes[i].kind == PD_NODE_FF) { ff = i; break; }
        int *const orig_ff_children = f.design.nodes[ff].children; // normally NULL
        const int orig_ff_nchildren = f.design.nodes[ff].nchildren;
        static int fake_child = 0;
        f.design.nodes[ff].children = &fake_child;
        f.design.nodes[ff].nchildren = 1;
        expect_rejected(orig, f.design, "sink", "an FF with children is rejected");
        // Restore before pd_free_design - it unconditionally frees nodes[i].children, and
        // &fake_child is not a heap pointer.
        f.design.nodes[ff].children = orig_ff_children;
        f.design.nodes[ff].nchildren = orig_ff_nchildren;
        pd_free_design(&f.design);
    }
    {
        SplitFixture f = build_split_fixture(testcase_dir);
        std::strncpy(f.design.nodes[f.new_node_id].name, "TOTALLY_UNEXPECTED_NAME", PD_MAX_NAME - 1);
        expect_rejected(orig, f.design, "unrecognized component",
                       "a new component not named NEW_BUF_X is rejected");
        pd_free_design(&f.design);
    }
    {
        SplitFixture f = build_split_fixture(testcase_dir);
        // Break relative order *without* breaking structural self-consistency: move one of the
        // new buffer's children to become a direct child of the root instead, updating both
        // sides of the link (old parent's children array, new parent's children array, and the
        // node's own parent field) so only the order-preservation check (not the bidirectional
        // parent/child check) can catch this.
        PdNode &new_buf = f.design.nodes[f.new_node_id];
        const int victim = new_buf.children[0];
        for (int k = 0; k < new_buf.nchildren; k++) {
            if (new_buf.children[k] == victim) {
                new_buf.children[k] = new_buf.children[new_buf.nchildren - 1];
                new_buf.nchildren--;
                break;
            }
        }
        PdNode &root = f.design.nodes[0];
        int *grown = (int *)malloc(sizeof(int) * static_cast<size_t>(root.nchildren + 1));
        std::memcpy(grown, root.children, sizeof(int) * static_cast<size_t>(root.nchildren));
        grown[root.nchildren] = victim;
        std::free(root.children);
        root.children = grown;
        root.nchildren++;
        f.design.nodes[victim].parent = 0;

        expect_rejected(orig, f.design, "order changed",
                       "moving an existing component out of its ancestor chain is rejected");
        pd_free_design(&f.design);
    }

    pd_free_design(&orig);

    std::cout << "\n" << g_pass << " passed, " << g_fail << " failed\n";
    return g_fail == 0 ? 0 : 1;
}

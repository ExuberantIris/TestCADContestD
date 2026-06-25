#include "pd_c_api.hpp"

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

/** First buffer node with at least `min_children` direct children. */
int find_buffer_with_children(const PdDesign &d, int min_children)
{
    for (int i = 0; i < d.n_nodes; i++)
        if (d.nodes[i].kind == PD_NODE_BUF && d.nodes[i].nchildren >= min_children)
            return i;
    return -1;
}

/** First buffer node with exactly one direct child - the zero cell's max_fanout is 1 (it's
 *  never evaluated at any other fanout, see pd_add_zero_cell), so assigning it to a
 *  multi-child buffer would trip the max_fanout check before ever reaching the zero-cell-leak
 *  check this test wants to exercise in isolation. */
int find_buffer_with_exactly_one_child(const PdDesign &d)
{
    for (int i = 0; i < d.n_nodes; i++)
        if (d.nodes[i].kind == PD_NODE_BUF && d.nodes[i].nchildren == 1)
            return i;
    return -1;
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

} // namespace

int main(int argc, char **argv)
{
    const std::string testcase_dir = resolve_testcase_dir(argc, argv);
    char err[512];

    std::cout << "=== unittest: pd_add_zero_cell / pd_remove_buffer ===\n";
    std::cout << "testcase_dir: " << testcase_dir << "\n\n";

    PdDesign orig{};
    if (pd_load_design(testcase_dir.c_str(), &orig, err, sizeof(err)) != 0) {
        std::cerr << "Load failed: " << err << "\n";
        return 1;
    }
    pd_annotate_clock(&orig);
    pd_compute_timing(&orig);

    PdDesign design{};
    if (pd_load_design(testcase_dir.c_str(), &design, err, sizeof(err)) != 0) {
        std::cerr << "Reload failed: " << err << "\n";
        pd_free_design(&orig);
        return 1;
    }

    const int parent = find_buffer_with_children(design, 2);
    if (parent < 0) {
        std::cerr << "No buffer with >=2 children found - cannot exercise this primitive\n";
        pd_free_design(&orig);
        pd_free_design(&design);
        return 2;
    }
    const int orig_parent_nchildren = design.nodes[parent].nchildren;
    const int chosen_child = design.nodes[parent].children[0];

    // --- Step 1: insert a decoupling buffer exactly like insert_decoupling_buffers would ----
    int new_node_id = -1;
    if (pd_insert_buffer_on_child(&design, parent, chosen_child, /*new_cell_idx=*/0, &new_node_id,
                                  err, sizeof(err)) != 0) {
        std::cerr << "pd_insert_buffer_on_child failed: " << err << "\n";
        pd_free_design(&orig);
        pd_free_design(&design);
        return 3;
    }
    std::printf("Inserted '%s' (id=%d) between '%s' and '%s'\n",
               design.nodes[new_node_id].name, new_node_id, design.nodes[parent].name,
               design.nodes[chosen_child].name);

    // --- Step 2: add the synthetic zero cell and resize the new buffer onto it --------------
    int zero_cell_idx = -1;
    if (pd_add_zero_cell(&design, &zero_cell_idx, err, sizeof(err)) != 0) {
        std::cerr << "pd_add_zero_cell failed: " << err << "\n";
        pd_free_design(&orig);
        pd_free_design(&design);
        return 4;
    }
    check(zero_cell_idx == design.n_cells - 1, "zero cell appended at the end of the library");
    check(pd_cell_is_zero(&design.cells[zero_cell_idx]), "pd_cell_is_zero recognizes it");
    for (int ci = 0; ci < design.n_cells - 1; ci++)
        check(!pd_cell_is_zero(&design.cells[ci]), "no real library cell is mistaken for zero");

    design.nodes[new_node_id].cell_idx = zero_cell_idx;
    std::strncpy(design.nodes[new_node_id].cell, design.cells[zero_cell_idx].name, PD_MAX_NAME - 1);
    design.nodes[new_node_id].cell[PD_MAX_NAME - 1] = '\0';

    pd_annotate_clock(&design);
    pd_compute_timing(&design);

    // A buffer on the zero cell contributes exactly zero delay and area - timing/area must
    // already be byte-identical to the un-inserted original, *before* any physical removal.
    check(std::fabs(design.total_area - orig.total_area) < 1e-9,
         "area with zero-cell buffer present already matches the un-inserted original");
    check(std::fabs(design.wns_setup_ss - orig.wns_setup_ss) < 1e-9,
         "WNS_ss with zero-cell buffer present already matches the un-inserted original");
    check(std::fabs(design.tns_setup_ss - orig.tns_setup_ss) < 1e-9,
         "TNS_ss with zero-cell buffer present already matches the un-inserted original");

    // --- Step 3: legality must reject it while it's still physically present ----------------
    {
        char e[512] = {0};
        const int rc = pd_check_legality(&orig, &design, e, sizeof(e));
        const bool rejected_for_zero_cell =
            rc != 0 && std::strstr(e, "zero-area placeholder") != nullptr;
        if (!rejected_for_zero_cell)
            std::cout << "    (rc=" << rc << " err=\"" << e << "\")\n";
        check(rejected_for_zero_cell,
             "legality rejects a NEW_BUF_* still assigned the zero-area placeholder cell");
    }

    // --- Step 4: physically remove it, exactly as main.cpp's Phase 3 cleanup does -----------
    if (pd_remove_buffer(&design, new_node_id, err, sizeof(err)) != 0) {
        std::cerr << "pd_remove_buffer failed: " << err << "\n";
        pd_free_design(&orig);
        pd_free_design(&design);
        return 5;
    }
    check(design.nodes[parent].nchildren == orig_parent_nchildren,
         "parent's child count is back to what it was before insertion");
    check(design.nodes[chosen_child].parent == parent,
         "chosen child's parent points straight back at the original parent");
    check(design.nodes[chosen_child].level == orig.nodes[chosen_child].level,
         "chosen child's level is back to its pre-insertion value");

    pd_annotate_clock(&design);
    pd_compute_timing(&design);
    check(std::fabs(design.total_area - orig.total_area) < 1e-9,
         "area after removal matches the un-inserted original");
    check(std::fabs(design.wns_setup_ss - orig.wns_setup_ss) < 1e-9,
         "WNS_ss after removal matches the un-inserted original");
    check(std::fabs(design.tns_setup_ss - orig.tns_setup_ss) < 1e-9,
         "TNS_ss after removal matches the un-inserted original");

    {
        char e[512] = {0};
        check(pd_check_legality(&orig, &design, e, sizeof(e)) == 0,
             "legality passes once the zero-cell buffer is physically removed");
    }

    // --- Step 5: an *existing* buffer carrying the zero cell must never be silently allowed -
    // (single-child victim: the zero cell's max_fanout is 1, so a multi-child buffer would be
    // rejected for exceeding max_fanout before ever reaching the zero-cell-leak check - using a
    // single-child buffer isolates the check this step actually wants to exercise.)
    {
        PdDesign d2{};
        if (pd_load_design(testcase_dir.c_str(), &d2, err, sizeof(err)) == 0) {
            int zci = -1;
            const int victim = find_buffer_with_exactly_one_child(d2);
            if (victim >= 0 && pd_add_zero_cell(&d2, &zci, err, sizeof(err)) == 0) {
                d2.nodes[victim].cell_idx = zci;
                char e[512] = {0};
                const int rc = pd_check_legality(&d2, &d2, e, sizeof(e));
                const bool rejected = rc != 0 && std::strstr(e, "zero-area placeholder") != nullptr;
                if (!rejected)
                    std::cout << "    (rc=" << rc << " err=\"" << e << "\")\n";
                check(rejected,
                     "an existing (non-NEW_BUF_*) buffer assigned the zero cell is rejected, "
                     "never silently deleted");
            } else if (victim < 0) {
                std::cout << "  [SKIP] no single-child buffer found to exercise this case\n";
            }
            pd_free_design(&d2);
        }
    }

    // --- Step 6: round-trip - write, reload with the unmodified parser, compare to orig -----
    const std::string result_dir = "result/testcase_remove_buffer_unittest";
    mkdir(result_dir.c_str(), 0755);
    const std::string out_path = result_dir + "/modified_clk_tree.structure";
    if (pd_write_structure(&design, out_path.c_str(), err, sizeof(err)) != 0) {
        std::cerr << "pd_write_structure failed: " << err << "\n";
        pd_free_design(&orig);
        pd_free_design(&design);
        return 6;
    }
    std::printf("Wrote %s\n", out_path.c_str());

    auto copy_file = [](const std::string &src, const std::string &dst) {
        FILE *in = std::fopen(src.c_str(), "rb");
        if (!in)
            return false;
        FILE *out = std::fopen(dst.c_str(), "wb");
        if (!out) {
            std::fclose(in);
            return false;
        }
        char buf[4096];
        size_t n;
        while ((n = std::fread(buf, 1, sizeof(buf), in)) > 0)
            std::fwrite(buf, 1, n, out);
        std::fclose(in);
        std::fclose(out);
        return true;
    };
    bool copy_ok = true;
    copy_ok &= copy_file(out_path, result_dir + "/clk_tree.structure");
    copy_ok &= copy_file(testcase_dir + "/buf.lib", result_dir + "/buf.lib");
    copy_ok &= copy_file(testcase_dir + "/SS_delay.rpt", result_dir + "/SS_delay.rpt");
    copy_ok &= copy_file(testcase_dir + "/FF_delay.rpt", result_dir + "/FF_delay.rpt");
    if (!copy_ok) {
        std::cerr << "Failed to stage round-trip testcase inputs\n";
        pd_free_design(&orig);
        pd_free_design(&design);
        return 7;
    }

    PdDesign reloaded{};
    if (pd_load_design(result_dir.c_str(), &reloaded, err, sizeof(err)) != 0) {
        std::cerr << "Round-trip reload FAILED: " << err << "\n";
        g_fail++;
    } else {
        pd_annotate_clock(&reloaded);
        pd_compute_timing(&reloaded);
        // The orphaned NEW_BUF_* never reached the output file at all, so a fresh parse of it
        // must come back with exactly the *original* node count - one fewer than `design`'s
        // in-memory n_nodes (which still counts the orphan's now-unreachable array slot).
        check(reloaded.n_nodes == orig.n_nodes,
             "round-trip node count matches the never-inserted original exactly (orphan dropped)");
        check(std::fabs(reloaded.total_area - orig.total_area) < 1e-9,
             "round-trip area matches the never-inserted original");
        check(std::fabs(reloaded.wns_setup_ss - orig.wns_setup_ss) < 1e-9,
             "round-trip WNS_ss matches the never-inserted original");
        check(std::fabs(reloaded.tns_setup_ss - orig.tns_setup_ss) < 1e-9,
             "round-trip TNS_ss matches the never-inserted original");
        pd_free_design(&reloaded);
    }

    pd_free_design(&orig);
    pd_free_design(&design);

    std::cout << "\n" << g_pass << " passed, " << g_fail << " failed\n";
    return g_fail == 0 ? 0 : 1;
}

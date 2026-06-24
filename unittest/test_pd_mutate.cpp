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
int find_splittable_buffer(const PdDesign &d, int min_children)
{
    for (int i = 0; i < d.n_nodes; i++) {
        if (d.nodes[i].kind == PD_NODE_BUF && d.nodes[i].nchildren >= min_children)
            return i;
    }
    return -1;
}

bool check(bool cond, const char *label)
{
    std::cout << (cond ? "  [PASS] " : "  [FAIL] ") << label << "\n";
    return cond;
}

} // namespace

int main(int argc, char **argv)
{
    const std::string testcase_dir = resolve_testcase_dir(argc, argv);
    char err[512];
    bool all_pass = true;

    std::cout << "=== unittest: pd_split_buffer ===\n";
    std::cout << "testcase_dir: " << testcase_dir << "\n\n";

    PdDesign design{};
    if (pd_load_design(testcase_dir.c_str(), &design, err, sizeof(err)) != 0) {
        std::cerr << "Load failed: " << err << "\n";
        return 1;
    }

    pd_annotate_clock(&design);
    pd_compute_timing(&design);

    const double area_before = design.total_area;
    const double wns_ss_before = design.wns_setup_ss;
    const double tns_ss_before = design.tns_setup_ss;
    const int n_nodes_before = design.n_nodes;

    const int target = find_splittable_buffer(design, 2);
    if (target < 0) {
        std::cerr << "No buffer with >=2 children found - cannot exercise split\n";
        pd_free_design(&design);
        return 2;
    }

    const int orig_nchildren = design.nodes[target].nchildren;
    std::vector<int> orig_children(design.nodes[target].children,
                                   design.nodes[target].children + orig_nchildren);
    const int orig_level = design.nodes[target].level;
    std::printf("Splitting node '%s' (id=%d, level=%d, %d children)\n",
               design.nodes[target].name, target, orig_level, orig_nchildren);

    // Pick any cell whose max_fanout covers the new buffer's fanout (it will drive all of the
    // split node's original children, same fanout the split node itself used to need).
    int new_cell_idx = -1;
    for (int i = 0; i < design.n_cells; i++) {
        if (design.cells[i].max_fanout >= orig_nchildren) {
            new_cell_idx = i;
            break;
        }
    }
    if (new_cell_idx < 0) {
        std::cerr << "No cell in buf.lib supports fanout " << orig_nchildren << "\n";
        pd_free_design(&design);
        return 3;
    }

    int new_node_id = -1;
    if (pd_split_buffer(&design, target, new_cell_idx, &new_node_id, err, sizeof(err)) != 0) {
        std::cerr << "pd_split_buffer failed: " << err << "\n";
        pd_free_design(&design);
        return 4;
    }
    std::printf("New node: '%s' (id=%d)\n\n", design.nodes[new_node_id].name, new_node_id);

    // --- Structural checks (in-memory) ----------------------------------------------------
    all_pass &= check(design.n_nodes == n_nodes_before + 1, "node count increased by exactly 1");
    all_pass &= check(design.nodes[target].nchildren == 1, "split node now has exactly 1 child");
    all_pass &= check(design.nodes[target].children[0] == new_node_id,
                      "split node's only child is the new buffer");
    all_pass &= check(design.nodes[new_node_id].nchildren == orig_nchildren,
                      "new buffer inherited the original child count");
    all_pass &= check(design.nodes[new_node_id].level == orig_level + 1,
                      "new buffer sits one level below the split node");
    all_pass &= check(std::string(design.nodes[new_node_id].name) == "NEW_BUF_0",
                      "new buffer named NEW_BUF_0");

    bool children_match = (design.nodes[new_node_id].nchildren == orig_nchildren);
    bool levels_bumped = true;
    for (int i = 0; i < orig_nchildren && children_match; i++) {
        if (design.nodes[new_node_id].children[i] != orig_children[static_cast<std::size_t>(i)])
            children_match = false;
        const PdNode &child = design.nodes[orig_children[static_cast<std::size_t>(i)]];
        if (child.parent != new_node_id)
            children_match = false;
        if (child.level != orig_level + 2)
            levels_bumped = false;
    }
    all_pass &= check(children_match, "new buffer's children == split node's original children, reparented");
    all_pass &= check(levels_bumped, "original children's subtree levels bumped by 1");

    // No duplicate names anywhere in the design.
    std::set<std::string> names;
    bool unique_names = true;
    for (int i = 0; i < design.n_nodes; i++) {
        if (!names.insert(design.nodes[i].name).second)
            unique_names = false;
    }
    all_pass &= check(unique_names, "all node names unique");

    pd_annotate_clock(&design);
    pd_compute_timing(&design);
    const double area_after_mutation = design.total_area;
    const double expected_area =
        area_before + design.cells[new_cell_idx].width * design.cells[new_cell_idx].height;
    all_pass &= check(std::fabs(area_after_mutation - expected_area) < 1e-9,
                      "total area increased by exactly the new cell's area");

    std::printf("\nTiming before split : WNS_ss=%.6f TNS_ss=%.6f area=%.6f\n", wns_ss_before,
               tns_ss_before, area_before);
    std::printf("Timing after split  : WNS_ss=%.6f TNS_ss=%.6f area=%.6f\n\n",
               design.wns_setup_ss, design.tns_setup_ss, design.total_area);

    // --- Round-trip check: write the mutated structure out, then re-parse it with the same
    // parser the contest grader would use, and confirm an independent read of our own output
    // reproduces identical timing. This is the strongest available check that the emitted file
    // is actually legal, not just that our in-memory mutation didn't crash. ------------------
    const std::string result_dir = "result/testcase_mutate_unittest";
    mkdir(result_dir.c_str(), 0755);
    const std::string out_path = result_dir + "/modified_clk_tree.structure";
    if (pd_write_structure(&design, out_path.c_str(), err, sizeof(err)) != 0) {
        std::cerr << "pd_write_structure failed: " << err << "\n";
        pd_free_design(&design);
        return 5;
    }
    std::printf("Wrote %s\n", out_path.c_str());

    // pd_load_design re-reads clk_tree.structure from the testcase dir, so point a throwaway
    // dir's worth of inputs at our freshly written structure file by reusing buf.lib/delay.rpt
    // from the original testcase and only swapping clk_tree.structure.
    const std::string roundtrip_dir = result_dir;
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
    // pd_load_design looks for clk_tree.structure specifically; our mutated output was written
    // as modified_clk_tree.structure (the contest's required output filename), so stage a copy
    // under the input filename the parser expects.
    copy_ok &= copy_file(out_path, roundtrip_dir + "/clk_tree.structure");
    copy_ok &= copy_file(testcase_dir + "/buf.lib", roundtrip_dir + "/buf.lib");
    copy_ok &= copy_file(testcase_dir + "/SS_delay.rpt", roundtrip_dir + "/SS_delay.rpt");
    copy_ok &= copy_file(testcase_dir + "/FF_delay.rpt", roundtrip_dir + "/FF_delay.rpt");
    if (!copy_ok) {
        std::cerr << "Failed to stage round-trip testcase inputs\n";
        pd_free_design(&design);
        return 6;
    }

    PdDesign reloaded{};
    if (pd_load_design(roundtrip_dir.c_str(), &reloaded, err, sizeof(err)) != 0) {
        std::cerr << "Round-trip reload FAILED: " << err << "\n";
        all_pass = false;
    } else {
        pd_annotate_clock(&reloaded);
        pd_compute_timing(&reloaded);
        all_pass &= check(reloaded.n_nodes == design.n_nodes, "round-trip node count matches");
        all_pass &= check(std::fabs(reloaded.total_area - design.total_area) < 1e-9,
                          "round-trip area matches");
        all_pass &= check(std::fabs(reloaded.wns_setup_ss - design.wns_setup_ss) < 1e-9,
                          "round-trip WNS_ss matches");
        all_pass &= check(std::fabs(reloaded.tns_setup_ss - design.tns_setup_ss) < 1e-9,
                          "round-trip TNS_ss matches");
        const int reloaded_new_id = [&]() {
            for (int i = 0; i < reloaded.n_nodes; i++)
                if (std::string(reloaded.nodes[i].name) == "NEW_BUF_0")
                    return i;
            return -1;
        }();
        all_pass &= check(reloaded_new_id >= 0, "round-trip reload finds NEW_BUF_0");
        pd_free_design(&reloaded);
    }

    pd_free_design(&design);

    std::cout << "\n" << (all_pass ? "PASS" : "FAIL") << "\n";
    return all_pass ? 0 : 1;
}

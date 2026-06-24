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
    for (int i = 0; i < d.n_nodes; i++)
        if (d.nodes[i].kind == PD_NODE_BUF && d.nodes[i].nchildren >= min_children)
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

    std::cout << "=== unittest: pd_insert_buffer_on_child ===\n";
    std::cout << "testcase_dir: " << testcase_dir << "\n\n";

    PdDesign orig{};
    if (pd_load_design(testcase_dir.c_str(), &orig, err, sizeof(err)) != 0) {
        std::cerr << "Load failed: " << err << "\n";
        return 1;
    }
    pd_annotate_clock(&orig);
    pd_compute_timing(&orig);
    const double area_before = orig.total_area;

    PdDesign design{};
    if (pd_load_design(testcase_dir.c_str(), &design, err, sizeof(err)) != 0) {
        std::cerr << "Reload failed: " << err << "\n";
        pd_free_design(&orig);
        return 1;
    }

    const int parent = find_splittable_buffer(design, 2);
    if (parent < 0) {
        std::cerr << "No buffer with >=2 children found - cannot exercise this primitive\n";
        pd_free_design(&orig);
        pd_free_design(&design);
        return 2;
    }
    const int orig_parent_nchildren = design.nodes[parent].nchildren;
    std::vector<int> siblings(design.nodes[parent].children,
                              design.nodes[parent].children + orig_parent_nchildren);
    const int chosen_child = siblings[0];
    std::vector<int> other_siblings(siblings.begin() + 1, siblings.end());
    const int chosen_child_orig_level = design.nodes[chosen_child].level;
    std::vector<int> other_sibling_orig_levels;
    for (int s : other_siblings)
        other_sibling_orig_levels.push_back(design.nodes[s].level);

    std::printf("Parent '%s' (id=%d) has %d children; inserting on child '%s' only\n",
               design.nodes[parent].name, parent, orig_parent_nchildren,
               design.nodes[chosen_child].name);

    int new_cell_idx = 0; // fanout 1 is supported by every cell in every testcase's library
    int new_node_id = -1;
    if (pd_insert_buffer_on_child(&design, parent, chosen_child, new_cell_idx, &new_node_id, err,
                                  sizeof(err)) != 0) {
        std::cerr << "pd_insert_buffer_on_child failed: " << err << "\n";
        pd_free_design(&orig);
        pd_free_design(&design);
        return 3;
    }
    std::printf("New node: '%s' (id=%d)\n\n", design.nodes[new_node_id].name, new_node_id);

    // --- Structural checks: the key differentiator vs pd_split_buffer is that siblings are
    // completely untouched. ------------------------------------------------------------------
    check(design.n_nodes == orig.n_nodes + 1, "node count increased by exactly 1");
    check(design.nodes[parent].nchildren == orig_parent_nchildren,
         "parent's child *count* is unchanged (1-for-1 substitution, not split)");
    check(design.nodes[new_node_id].nchildren == 1, "new buffer drives exactly 1 child");
    check(design.nodes[new_node_id].children[0] == chosen_child,
         "new buffer's one child is the chosen child");
    check(design.nodes[chosen_child].parent == new_node_id,
         "chosen child's parent is now the new buffer");
    check(design.nodes[new_node_id].level == chosen_child_orig_level,
         "new buffer occupies the chosen child's old level");
    check(design.nodes[chosen_child].level == chosen_child_orig_level + 1,
         "chosen child's level bumped by exactly 1");

    bool siblings_untouched = true;
    for (std::size_t i = 0; i < other_siblings.size(); i++) {
        const PdNode &sib = design.nodes[other_siblings[i]];
        if (sib.parent != parent || sib.level != other_sibling_orig_levels[i])
            siblings_untouched = false;
    }
    check(siblings_untouched, "every other sibling's parent and level are completely untouched");

    bool parent_still_lists_all = (design.nodes[parent].nchildren == orig_parent_nchildren);
    if (parent_still_lists_all) {
        // parent's children should be: new_node_id in chosen_child's old slot, all other
        // siblings unchanged and in the same relative positions.
        for (int i = 0; i < orig_parent_nchildren; i++) {
            const int c = design.nodes[parent].children[i];
            const int expected = (siblings[static_cast<std::size_t>(i)] == chosen_child)
                                     ? new_node_id
                                     : siblings[static_cast<std::size_t>(i)];
            if (c != expected)
                parent_still_lists_all = false;
        }
    }
    check(parent_still_lists_all,
         "parent's children array correctly swaps in the new buffer at the same slot");

    std::set<std::string> names;
    bool unique_names = true;
    for (int i = 0; i < design.n_nodes; i++)
        if (!names.insert(design.nodes[i].name).second)
            unique_names = false;
    check(unique_names, "all node names unique");

    pd_annotate_clock(&design);
    pd_compute_timing(&design);
    const double expected_area =
        area_before + design.cells[new_cell_idx].width * design.cells[new_cell_idx].height;
    check(std::fabs(design.total_area - expected_area) < 1e-9,
         "total area increased by exactly the new cell's area");

    std::printf("\nTiming before insert : WNS_ss=%.6f TNS_ss=%.6f area=%.6f\n", orig.wns_setup_ss,
               orig.tns_setup_ss, area_before);
    std::printf("Timing after insert  : WNS_ss=%.6f TNS_ss=%.6f area=%.6f\n\n",
               design.wns_setup_ss, design.tns_setup_ss, design.total_area);

    // --- Legality check -----------------------------------------------------------------
    {
        char e[512];
        check(pd_check_legality(&orig, &design, e, sizeof(e)) == 0,
             "result passes pd_check_legality");
    }

    // --- Round-trip check: write -> reload with the unmodified parser -> identical timing --
    const std::string result_dir = "result/testcase_insert_on_child_unittest";
    mkdir(result_dir.c_str(), 0755);
    const std::string out_path = result_dir + "/modified_clk_tree.structure";
    if (pd_write_structure(&design, out_path.c_str(), err, sizeof(err)) != 0) {
        std::cerr << "pd_write_structure failed: " << err << "\n";
        pd_free_design(&orig);
        pd_free_design(&design);
        return 4;
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
        return 5;
    }

    PdDesign reloaded{};
    if (pd_load_design(result_dir.c_str(), &reloaded, err, sizeof(err)) != 0) {
        std::cerr << "Round-trip reload FAILED: " << err << "\n";
        g_fail++;
    } else {
        pd_annotate_clock(&reloaded);
        pd_compute_timing(&reloaded);
        check(reloaded.n_nodes == design.n_nodes, "round-trip node count matches");
        check(std::fabs(reloaded.total_area - design.total_area) < 1e-9,
             "round-trip area matches");
        check(std::fabs(reloaded.wns_setup_ss - design.wns_setup_ss) < 1e-9,
             "round-trip WNS_ss matches");
        check(std::fabs(reloaded.tns_setup_ss - design.tns_setup_ss) < 1e-9,
             "round-trip TNS_ss matches");
        pd_free_design(&reloaded);
    }

    pd_free_design(&orig);
    pd_free_design(&design);

    std::cout << "\n" << g_pass << " passed, " << g_fail << " failed\n";
    return g_fail == 0 ? 0 : 1;
}

#include "insert_select.hpp"

#include <cstdio>
#include <set>
#include <utility>

bool pd_find_branch_point(const PdDesign *d, int ff_id, int *out_ancestor, int *out_child)
{
    int child = ff_id;
    int cur = d->nodes[ff_id].parent;
    while (cur >= 0 && d->nodes[cur].kind == PD_NODE_BUF) {
        if (d->nodes[cur].nchildren > 1) {
            *out_ancestor = cur;
            *out_child = child;
            return true;
        }
        child = cur;
        cur = d->nodes[cur].parent;
    }
    return false;
}

int insert_decoupling_buffers(PdDesign *d, char *err, std::size_t err_sz)
{
    if (!d) {
        if (err && err_sz > 0)
            std::snprintf(err, err_sz, "insert_decoupling_buffers: null design");
        return -1;
    }

    // (ancestor, child) edges at the nearest true branch point above each violating path's
    // launch/capture FF. std::set both dedupes and gives deterministic order.
    std::set<std::pair<int, int>> candidates;
    for (int p = 0; p < d->n_paths; p++) {
        const PdPath &path = d->paths[p];
        if (path.launch_id < 0 || path.capture_id < 0)
            continue;
        if (!(path.slack_setup_ss < 0.0) && !(path.slack_hold_ff < 0.0))
            continue;

        const int ff_ids[2] = {path.launch_id, path.capture_id};
        for (int ff_id : ff_ids) {
            int ancestor, child;
            if (pd_find_branch_point(d, ff_id, &ancestor, &child))
                candidates.insert({ancestor, child});
        }
    }

    if (candidates.empty())
        return 0;

    int smallest_cell = 0;
    for (int i = 1; i < d->n_cells; i++) {
        const double area = d->cells[i].width * d->cells[i].height;
        const double best_area = d->cells[smallest_cell].width * d->cells[smallest_cell].height;
        if (area < best_area)
            smallest_cell = i;
    }

    int inserted = 0;
    for (const auto &edge : candidates) {
        int new_node_id = -1;
        char insert_err[256];
        if (pd_insert_buffer_on_child(d, edge.first, edge.second, smallest_cell, &new_node_id,
                                      insert_err, sizeof(insert_err)) == 0) {
            inserted++;
        } else {
            std::fprintf(stderr, "insert_decoupling_buffers: skipped edge (%d,%d): %s\n",
                        edge.first, edge.second, insert_err);
        }
    }

    pd_annotate_clock(d);
    pd_compute_timing(d);
    return inserted;
}

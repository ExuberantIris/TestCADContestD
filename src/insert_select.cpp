#include "insert_select.hpp"

#include <cstdio>
#include <set>
#include <utility>

int insert_decoupling_buffers(PdDesign *d, char *err, std::size_t err_sz)
{
    if (!d) {
        if (err && err_sz > 0)
            std::snprintf(err, err_sz, "insert_decoupling_buffers: null design");
        return -1;
    }

    // (parent_id, ff_id) edges where a violating path's launch/capture FF is a direct child of
    // a buffer it shares with siblings. std::set both dedupes and gives deterministic order.
    std::set<std::pair<int, int>> candidates;
    for (int p = 0; p < d->n_paths; p++) {
        const PdPath &path = d->paths[p];
        if (path.launch_id < 0 || path.capture_id < 0)
            continue;
        if (!(path.slack_setup_ss < 0.0) && !(path.slack_hold_ff < 0.0))
            continue;

        const int ff_ids[2] = {path.launch_id, path.capture_id};
        for (int ff_id : ff_ids) {
            const int parent_id = d->nodes[ff_id].parent;
            if (parent_id < 0)
                continue;
            if (d->nodes[parent_id].kind == PD_NODE_BUF && d->nodes[parent_id].nchildren > 1)
                candidates.insert({parent_id, ff_id});
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

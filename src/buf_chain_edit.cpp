#include "buf_chain_edit.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>

static int ensure_nodes(PdDesign *d)
{
    if (d->n_nodes < d->cap_nodes)
        return 0;
    const int new_cap = d->cap_nodes ? d->cap_nodes * 2 : PD_INIT_NODES;
    PdNode *p = static_cast<PdNode *>(std::realloc(d->nodes, static_cast<std::size_t>(new_cap) * sizeof(PdNode)));
    if (!p)
        return -1;
    d->nodes = p;
    d->cap_nodes = new_cap;
    return 0;
}

static void bump_subtree_levels(PdDesign *d, int node_id, int delta)
{
    if (delta == 0)
        return;
    PdNode *n = &d->nodes[node_id];
    n->level += delta;
    for (int i = 0; i < n->nchildren; i++)
        bump_subtree_levels(d, n->children[i], delta);
}

static void replace_parent_child(PdDesign *d, int parent_id, int old_child, int new_child)
{
    PdNode *parent = &d->nodes[parent_id];
    for (int i = 0; i < parent->nchildren; i++) {
        if (parent->children[i] == old_child) {
            parent->children[i] = new_child;
            return;
        }
    }
}

static int append_single_child(PdDesign *d, int parent_id, int child_id)
{
    PdNode *parent = &d->nodes[parent_id];
    int *kids = static_cast<int *>(std::realloc(parent->children,
                                                static_cast<std::size_t>(parent->nchildren + 1) *
                                                    sizeof(int)));
    if (!kids)
        return -1;
    parent->children = kids;
    parent->children[parent->nchildren++] = child_id;
    d->nodes[child_id].parent = parent_id;
    return 0;
}

static int alloc_buf_node(PdDesign *d, int cell_idx, int level, const char *name)
{
    if (ensure_nodes(d) != 0)
        return -1;

    const int id = d->n_nodes++;
    PdNode *node = &d->nodes[id];
    std::memset(node, 0, sizeof(*node));
    node->id = id;
    node->kind = PD_NODE_BUF;
    node->level = level;
    node->parent = -1;
    node->cell_idx = cell_idx;
    std::strncpy(node->name, name, PD_MAX_NAME - 1);
    node->name[PD_MAX_NAME - 1] = '\0';
    std::strncpy(node->cell, d->cells[cell_idx].name, PD_MAX_NAME - 1);
    node->cell[PD_MAX_NAME - 1] = '\0';
    return id;
}

static int prefix_entry_child(const PdDesign *d, int parent_id, int buf_id)
{
    const PdNode *parent = &d->nodes[parent_id];
    for (int i = 0; i < parent->nchildren; i++) {
        int walk = parent->children[i];
        const int entry = walk;
        while (walk >= 0) {
            if (walk == buf_id)
                return entry;
            const PdNode *n = &d->nodes[walk];
            if (n->kind != PD_NODE_BUF || std::strncmp(n->name, "NEW_BUF_", 8) != 0 ||
                n->nchildren != 1)
                break;
            walk = n->children[0];
        }
    }
    return -1;
}

static void collect_prefix_cells(const PdDesign *d, int parent_id, int buf_id,
                                 std::vector<int> *prefix_cells)
{
    prefix_cells->clear();
    const int entry = prefix_entry_child(d, parent_id, buf_id);
    if (entry < 0 || entry == buf_id)
        return;

    int walk = entry;
    while (walk != buf_id) {
        const PdNode *n = &d->nodes[walk];
        if (n->kind != PD_NODE_BUF || std::strncmp(n->name, "NEW_BUF_", 8) != 0)
            break;
        prefix_cells->push_back(n->cell_idx);
        if (n->nchildren != 1)
            break;
        walk = n->children[0];
    }
}

static void remove_live_prefix(PdDesign *d, const LpBranch &br)
{
    std::vector<int> prefix_cells;
    collect_prefix_cells(d, br.parent_node, br.child_node, &prefix_cells);
    if (prefix_cells.empty())
        return;

    const int entry = prefix_entry_child(d, br.parent_node, br.child_node);
    if (entry < 0 || entry == br.child_node)
        return;

    replace_parent_child(d, br.parent_node, entry, br.child_node);
    d->nodes[br.child_node].parent = br.parent_node;
    bump_subtree_levels(d, br.child_node, -static_cast<int>(prefix_cells.size()));
}

static int insert_prefix_nodes(PdDesign *d, int parent_id, int child_id,
                               const std::vector<int> &prefix_cells, int *next_new_buf_id)
{
    const int k = static_cast<int>(prefix_cells.size());
    if (k <= 0)
        return 0;

    bump_subtree_levels(d, child_id, k);

    const int base_level = d->nodes[parent_id].level;
    int first_new = -1;
    int prev_new = -1;

    for (int i = 0; i < k; i++) {
        char name[PD_MAX_NAME];
        std::snprintf(name, sizeof(name), "NEW_BUF_%d", (*next_new_buf_id)++);
        const int nid =
            alloc_buf_node(d, prefix_cells[static_cast<std::size_t>(i)], base_level + 1 + i, name);
        if (nid < 0)
            return -1;

        if (first_new < 0)
            first_new = nid;
        if (prev_new >= 0 && append_single_child(d, prev_new, nid) != 0)
            return -1;
        prev_new = nid;
    }

    replace_parent_child(d, parent_id, child_id, first_new);
    d->nodes[first_new].parent = parent_id;
    if (append_single_child(d, prev_new, child_id) != 0)
        return -1;
    return 0;
}

int buf_chain_next_new_buf_id(const PdDesign *d)
{
    int next_id = 0;
    for (int i = 0; i < d->n_nodes; i++) {
        const char *name = d->nodes[i].name;
        if (std::strncmp(name, "NEW_BUF_", 8) != 0)
            continue;
        const int id = std::atoi(name + 8);
        if (id >= next_id)
            next_id = id + 1;
    }
    return next_id;
}

void buf_chain_capture_branch(const PdDesign *d, const LpBranch &br, BranchChainState *st)
{
    st->prefix_cells.clear();
    collect_prefix_cells(d, br.parent_node, br.child_node, &st->prefix_cells);
    st->final_cell_idx = d->nodes[br.child_node].cell_idx;
}

int buf_chain_set_branch(PdDesign *d, const LpBranch &br, const BranchChainState &target,
                         int *next_new_buf_id)
{
    if (target.final_cell_idx < 0)
        return -1;

    remove_live_prefix(d, br);

    if (!target.prefix_cells.empty()) {
        if (insert_prefix_nodes(d, br.parent_node, br.child_node, target.prefix_cells,
                                next_new_buf_id) != 0)
            return -1;
    }

    PdNode *buf = &d->nodes[br.child_node];
    if (buf->kind != PD_NODE_BUF)
        return -1;
    buf->cell_idx = target.final_cell_idx;
    std::strncpy(buf->cell, d->cells[target.final_cell_idx].name, PD_MAX_NAME - 1);
    buf->cell[PD_MAX_NAME - 1] = '\0';
    return 0;
}

const LpBufferChainEntry &buf_chain_lookup_nearest(const LpBufferChainDp *dp, int fanout,
                                                   double delay)
{
    const LpBufferChainEntry &exact = dp->lookup(fanout, delay);
    if (exact.reachable)
        return exact;

    const std::vector<double> &grid = dp->delays_for_fanout(fanout);
    if (grid.empty())
        return exact;

    double best_dist = std::numeric_limits<double>::infinity();
    const LpBufferChainEntry *best = nullptr;
    for (double d : grid) {
        const LpBufferChainEntry &e = dp->lookup(fanout, d);
        if (!e.reachable)
            continue;
        const double dist = std::fabs(d - delay);
        if (dist < best_dist) {
            best_dist = dist;
            best = &e;
        }
    }
    if (best)
        return *best;
    return exact;
}

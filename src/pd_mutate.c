#include "pd_mutate.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int fail(char *err, size_t err_sz, const char *msg)
{
    if (err && err_sz > 0)
        snprintf(err, err_sz, "%s", msg);
    return -1;
}

static int ensure_node_capacity(PdDesign *d, char *err, size_t err_sz)
{
    PdNode *p;
    int new_cap;

    if (d->n_nodes < d->cap_nodes)
        return 0;
    new_cap = d->cap_nodes ? d->cap_nodes * 2 : 256;
    p = realloc(d->nodes, (size_t)new_cap * sizeof(PdNode));
    if (!p)
        return fail(err, err_sz, "pd_mutate: out of memory (nodes)");
    d->nodes = p;
    d->cap_nodes = new_cap;
    return 0;
}

static void bump_level_recursive(PdDesign *d, int node_id, int delta)
{
    PdNode *n = &d->nodes[node_id];
    int i;

    n->level += delta;
    for (i = 0; i < n->nchildren; i++)
        bump_level_recursive(d, n->children[i], delta);
}

int pd_split_buffer(PdDesign *d, int buf_node_id, int new_cell_idx, int *out_new_node_id,
                    char *err, size_t err_sz)
{
    PdNode *b;
    PdNode *new_node;
    int new_id;
    int i;

    if (!d || buf_node_id < 0 || buf_node_id >= d->n_nodes)
        return fail(err, err_sz, "pd_split_buffer: invalid node id");
    if (d->nodes[buf_node_id].kind != PD_NODE_BUF)
        return fail(err, err_sz, "pd_split_buffer: target node is not a buffer");
    if (new_cell_idx < 0 || new_cell_idx >= d->n_cells)
        return fail(err, err_sz, "pd_split_buffer: invalid cell index");

    /* Grow d->nodes *before* taking any pointer into it - realloc may move the array. */
    if (ensure_node_capacity(d, err, err_sz) != 0)
        return -1;

    /* Allocate the split buffer's new (single-element) children array up front, before
     * mutating anything, so a failure here leaves the design untouched instead of requiring a
     * partial-mutation rollback. */
    int *new_b_children = (int *)malloc(sizeof(int));
    if (!new_b_children)
        return fail(err, err_sz, "pd_split_buffer: out of memory (children)");

    b = &d->nodes[buf_node_id];
    new_id = d->n_nodes;
    new_node = &d->nodes[new_id];

    memset(new_node, 0, sizeof(*new_node));
    new_node->id = new_id;
    snprintf(new_node->name, PD_MAX_NAME, "NEW_BUF_%d", d->next_new_buf_id++);
    strncpy(new_node->cell, d->cells[new_cell_idx].name, PD_MAX_NAME - 1);
    new_node->cell[PD_MAX_NAME - 1] = '\0';
    new_node->kind = PD_NODE_BUF;
    new_node->cell_idx = new_cell_idx;
    new_node->parent = buf_node_id;
    new_node->level = b->level + 1;
    new_node->is_sink = 0;

    /* New buffer takes over every child the split buffer used to drive directly. */
    new_node->children = b->children;
    new_node->nchildren = b->nchildren;
    for (i = 0; i < new_node->nchildren; i++) {
        d->nodes[new_node->children[i]].parent = new_id;
        bump_level_recursive(d, new_node->children[i], 1);
    }

    /* The split buffer now drives only the new buffer. */
    new_b_children[0] = new_id;
    b->children = new_b_children;
    b->nchildren = 1;

    d->n_nodes++;
    if (out_new_node_id)
        *out_new_node_id = new_id;
    return 0;
}

int pd_insert_buffer_on_child(PdDesign *d, int parent_id, int child_id, int new_cell_idx,
                              int *out_new_node_id, char *err, size_t err_sz)
{
    PdNode *p;
    PdNode *new_node;
    int new_id;
    int slot;
    int i;

    if (!d || parent_id < 0 || parent_id >= d->n_nodes)
        return fail(err, err_sz, "pd_insert_buffer_on_child: invalid parent id");
    if (d->nodes[parent_id].kind != PD_NODE_BUF)
        return fail(err, err_sz, "pd_insert_buffer_on_child: parent is not a buffer");
    if (child_id < 0 || child_id >= d->n_nodes)
        return fail(err, err_sz, "pd_insert_buffer_on_child: invalid child id");
    if (new_cell_idx < 0 || new_cell_idx >= d->n_cells)
        return fail(err, err_sz, "pd_insert_buffer_on_child: invalid cell index");

    p = &d->nodes[parent_id];
    slot = -1;
    for (i = 0; i < p->nchildren; i++) {
        if (p->children[i] == child_id) {
            slot = i;
            break;
        }
    }
    if (slot < 0)
        return fail(err, err_sz,
                    "pd_insert_buffer_on_child: child is not directly driven by parent");
    if (d->nodes[child_id].parent != parent_id)
        return fail(err, err_sz, "pd_insert_buffer_on_child: parent/child link inconsistent");

    /* Grow d->nodes *before* taking any pointer into it - realloc may move the array. */
    if (ensure_node_capacity(d, err, err_sz) != 0)
        return -1;

    /* Allocate the new buffer's single-element children array up front, before mutating
     * anything, so a failure here leaves the design untouched. */
    int *new_children = (int *)malloc(sizeof(int));
    if (!new_children)
        return fail(err, err_sz, "pd_insert_buffer_on_child: out of memory (children)");

    p = &d->nodes[parent_id]; /* re-fetch: ensure_node_capacity may have realloc'd d->nodes */
    new_id = d->n_nodes;
    new_node = &d->nodes[new_id];

    memset(new_node, 0, sizeof(*new_node));
    new_node->id = new_id;
    snprintf(new_node->name, PD_MAX_NAME, "NEW_BUF_%d", d->next_new_buf_id++);
    strncpy(new_node->cell, d->cells[new_cell_idx].name, PD_MAX_NAME - 1);
    new_node->cell[PD_MAX_NAME - 1] = '\0';
    new_node->kind = PD_NODE_BUF;
    new_node->cell_idx = new_cell_idx;
    new_node->parent = parent_id;
    new_node->level = d->nodes[child_id].level; /* takes over the child's old position */
    new_node->children = new_children;
    new_node->children[0] = child_id;
    new_node->nchildren = 1;

    /* Splice in: parent's *one* affected slot now points at the new buffer instead of
     * directly at child_id - every other child (sibling) of parent is left completely
     * untouched, which is the whole point relative to pd_split_buffer. */
    p->children[slot] = new_id;
    d->nodes[child_id].parent = new_id;
    bump_level_recursive(d, child_id, 1);

    d->n_nodes++;
    if (out_new_node_id)
        *out_new_node_id = new_id;
    return 0;
}

int pd_remove_buffer(PdDesign *d, int buf_node_id, char *err, size_t err_sz)
{
    PdNode *b;
    PdNode *p;
    int parent_id, child_id;
    int slot, i;

    if (!d || buf_node_id < 0 || buf_node_id >= d->n_nodes)
        return fail(err, err_sz, "pd_remove_buffer: invalid node id");

    b = &d->nodes[buf_node_id];
    if (b->kind != PD_NODE_BUF)
        return fail(err, err_sz, "pd_remove_buffer: target node is not a buffer");
    if (b->nchildren != 1)
        return fail(err, err_sz, "pd_remove_buffer: only single-child buffers can be removed");
    if (b->parent < 0 || b->parent >= d->n_nodes)
        return fail(err, err_sz, "pd_remove_buffer: buffer has no parent");

    parent_id = b->parent;
    child_id = b->children[0];
    p = &d->nodes[parent_id];

    slot = -1;
    for (i = 0; i < p->nchildren; i++) {
        if (p->children[i] == buf_node_id) {
            slot = i;
            break;
        }
    }
    if (slot < 0)
        return fail(err, err_sz, "pd_remove_buffer: parent/child link inconsistent");

    /* Splice out: parent's one slot now points straight at the (former) grandchild, and that
     * child's whole subtree shifts up one level to take the removed buffer's old position. */
    p->children[slot] = child_id;
    d->nodes[child_id].parent = parent_id;
    bump_level_recursive(d, child_id, -1);

    free(b->children);
    b->children = NULL;
    b->nchildren = 0;
    b->parent = -1;

    return 0;
}

int pd_add_zero_cell(PdDesign *d, int *out_cell_idx, char *err, size_t err_sz)
{
    PdCell *c;
    int idx;
    int i;

    if (!d)
        return fail(err, err_sz, "pd_add_zero_cell: null design");
    if (d->n_cells >= PD_MAX_CELLS)
        return fail(err, err_sz, "pd_add_zero_cell: cell library full");

    idx = d->n_cells;
    c = &d->cells[idx];
    memset(c, 0, sizeof(*c));
    snprintf(c->name, PD_MAX_NAME, "__ZERO_CELL__");
    c->width = 0.0;
    c->height = 0.0;
    c->max_fanout = PD_MAX_FANOUT_TBL;
    for (i = 0; i < PD_MAX_FANOUT_TBL; i++) {
        c->ss_delay[i] = 0.0;
        c->ff_delay[i] = 0.0;
    }

    d->n_cells = idx + 1;
    if (out_cell_idx)
        *out_cell_idx = idx;
    return 0;
}

int pd_cell_is_zero(const PdCell *c)
{
    return c->width <= 0.0 && c->height <= 0.0;
}

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
        return fail(err, err_sz, "pd_split_buffer: out of memory (nodes)");
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

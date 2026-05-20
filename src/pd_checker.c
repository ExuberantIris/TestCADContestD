#include "pd_checker.h"

#include "pd_util.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int check_unique_names(const PdDesign *d, char *err, size_t err_sz)
{
    int i, j;

    for (i = 0; i < d->n_nodes; i++) {
        for (j = i + 1; j < d->n_nodes; j++) {
            if (pd_streq(d->nodes[i].name, d->nodes[j].name)) {
                snprintf(err, err_sz, "duplicate node name: %s", d->nodes[i].name);
                return -1;
            }
        }
    }
    return 0;
}

static int check_ff_single_driver(const PdDesign *d, char *err, size_t err_sz)
{
    int i;

    for (i = 0; i < d->n_nodes; i++) {
        const PdNode *n = &d->nodes[i];
        if (n->kind != PD_NODE_FF)
            continue;
        if (n->parent < 0) {
            snprintf(err, err_sz, "FF %s has no driver", n->name);
            return -1;
        }
        if (d->nodes[n->parent].kind == PD_NODE_ROOT && n->parent != 0) {
            /* allowed: direct child of root only if root drives - unusual */
        }
    }
    return 0;
}

static int check_fanout(const PdDesign *d, char *err, size_t err_sz)
{
    int i;

    for (i = 0; i < d->n_nodes; i++) {
        const PdNode *n = &d->nodes[i];
        const PdCell *c;

        if (n->kind != PD_NODE_BUF)
            continue;
        c = &d->cells[n->cell_idx];
        if (n->fanout > c->max_fanout) {
            snprintf(err, err_sz, "buffer %s fanout %d exceeds max %d",
                     n->name, n->fanout, c->max_fanout);
            return -1;
        }
        if (n->nchildren == 0) {
            snprintf(err, err_sz, "buffer %s has no fanout pin connection", n->name);
            return -1;
        }
    }
    return 0;
}

static void compute_levels(const PdDesign *d, int *levels)
{
    int i;

    for (i = 0; i < d->n_nodes; i++)
        levels[i] = -1;

    if (d->n_nodes > 0)
        levels[0] = 0;

    for (i = 1; i < d->n_nodes; i++) {
        const PdNode *n = &d->nodes[i];
        if (n->parent >= 0 && levels[n->parent] >= 0)
            levels[i] = levels[n->parent] + 1;
    }
}

static int check_levels(const PdDesign *d, char *err, size_t err_sz)
{
    int *levels;
    int i;
    int rc = 0;

    levels = calloc((size_t)d->n_nodes, sizeof(int));
    if (!levels) {
        snprintf(err, err_sz, "out of memory");
        return -1;
    }

    compute_levels(d, levels);
    for (i = 0; i < d->n_nodes; i++) {
        if (levels[i] != d->nodes[i].level) {
            snprintf(err, err_sz, "level mismatch on %s: file=%d computed=%d",
                     d->nodes[i].name, d->nodes[i].level, levels[i]);
            rc = -1;
            break;
        }
    }
    free(levels);
    return rc;
}

static int check_topology_order(const PdDesign *d, char *err, size_t err_sz)
{
    int i, c;

    for (i = 0; i < d->n_nodes; i++) {
        const PdNode *n = &d->nodes[i];
        for (c = 0; c < n->nchildren; c++) {
            int ch = n->children[c];
            if (d->nodes[ch].parent != i) {
                snprintf(err, err_sz, "parent pointer mismatch for %s",
                         d->nodes[ch].name);
                return -1;
            }
        }
    }
    return 0;
}

int pd_check_legality(const PdDesign *d, char *err, size_t err_sz)
{
    if (check_unique_names(d, err, err_sz) != 0)
        return -1;
    if (check_ff_single_driver(d, err, err_sz) != 0)
        return -1;
    if (check_fanout(d, err, err_sz) != 0)
        return -1;
    if (check_levels(d, err, err_sz) != 0)
        return -1;
    if (check_topology_order(d, err, err_sz) != 0)
        return -1;
    if (err && err_sz > 0)
        snprintf(err, err_sz, "OK");
    return 0;
}

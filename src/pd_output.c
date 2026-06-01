#include "pd_output.h"

#include <stdio.h>

static void write_node_recursive(FILE *fp, const PdDesign *d, int node_id)
{
    const PdNode *n = &d->nodes[node_id];
    int i;

    if (n->kind == PD_NODE_ROOT) {
        fprintf(fp, "Root: %s\n", n->name);
    } else if (n->kind == PD_NODE_BUF) {
        fprintf(fp, "[%d] %s (%s)\n", n->level, n->name, n->cell);
    } else if (n->kind == PD_NODE_FF) {
        if (n->is_sink)
            fprintf(fp, "[%d] %s (%s) (SINK)\n", n->level, n->name, n->cell);
        else
            fprintf(fp, "[%d] %s (%s)\n", n->level, n->name, n->cell);
    }

    for (i = 0; i < n->nchildren; i++)
        write_node_recursive(fp, d, n->children[i]);
}

int pd_write_structure(const PdDesign *d, const char *out_path, char *err, size_t err_sz)
{
    FILE *fp = fopen(out_path, "w");

    if (!fp) {
        if (err && err_sz > 0)
            snprintf(err, err_sz, "cannot open output: %s", out_path);
        return -1;
    }

    if (d->n_nodes > 0)
        write_node_recursive(fp, d, 0);

    fclose(fp);
    return 0;
}

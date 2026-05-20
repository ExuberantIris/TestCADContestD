#include "pd_clock.h"

static double cell_delay(const PdCell *c, int fanout, int use_ss)
{
    int idx;
    const double *tbl;

    if (fanout < 1)
        fanout = 1;
    if (fanout > c->max_fanout)
        fanout = c->max_fanout;
    idx = fanout - 1;
    tbl = use_ss ? c->ss_delay : c->ff_delay;
    return tbl[idx];
}

static void annotate_subtree(PdDesign *d, int node_id, double acc_ss, double acc_ff)
{
    PdNode *n = &d->nodes[node_id];
    int i;

    if (n->kind == PD_NODE_BUF) {
        const PdCell *c = &d->cells[n->cell_idx];
        double dss, dff;

        n->fanout = n->nchildren > 0 ? n->nchildren : 1;
        dss = cell_delay(c, n->fanout, 1);
        dff = cell_delay(c, n->fanout, 0);
        acc_ss += dss;
        acc_ff += dff;
        n->area = c->width * c->height;
    } else if (n->kind == PD_NODE_FF) {
        n->d_clk_ss = acc_ss;
        n->d_clk_ff = acc_ff;
        return;
    } else if (n->kind == PD_NODE_ROOT) {
        n->fanout = n->nchildren;
    }

    for (i = 0; i < n->nchildren; i++)
        annotate_subtree(d, n->children[i], acc_ss, acc_ff);
}

void pd_annotate_clock(PdDesign *d)
{
    int i;

    if (d->n_nodes == 0)
        return;
    annotate_subtree(d, 0, 0.0, 0.0);

    d->total_area = 0.0;
    for (i = 0; i < d->n_nodes; i++) {
        if (d->nodes[i].kind == PD_NODE_BUF)
            d->total_area += d->nodes[i].area;
    }
}

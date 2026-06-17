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

static void recompute_total_area(PdDesign *d)
{
    int i;

    d->total_area = 0.0;
    for (i = 0; i < d->n_nodes; i++) {
        if (d->nodes[i].kind == PD_NODE_BUF)
            d->total_area += d->nodes[i].area;
    }
}

static int acc_at_node(const PdDesign *d, int target, int cur, double acc_ss, double acc_ff,
                       double *out_ss, double *out_ff)
{
    const PdNode *n = &d->nodes[cur];
    double a_ss = acc_ss;
    double a_ff = acc_ff;
    int i;

    if (cur == target) {
        *out_ss = a_ss;
        *out_ff = a_ff;
        return 1;
    }

    if (n->kind == PD_NODE_BUF) {
        const PdCell *c = &d->cells[n->cell_idx];
        const int fo = n->nchildren > 0 ? n->nchildren : 1;

        a_ss += cell_delay(c, fo, 1);
        a_ff += cell_delay(c, fo, 0);
    }

    for (i = 0; i < n->nchildren; i++) {
        if (acc_at_node(d, target, n->children[i], a_ss, a_ff, out_ss, out_ff))
            return 1;
    }
    return 0;
}

void pd_annotate_clock(PdDesign *d)
{
    if (d->n_nodes == 0)
        return;
    annotate_subtree(d, 0, 0.0, 0.0);
    recompute_total_area(d);
}

void pd_annotate_clock_subtree(PdDesign *d, int node_id)
{
    double acc_ss = 0.0;
    double acc_ff = 0.0;

    if (d->n_nodes == 0)
        return;
    if (node_id <= 0) {
        pd_annotate_clock(d);
        return;
    }
    if (!acc_at_node(d, node_id, 0, 0.0, 0.0, &acc_ss, &acc_ff))
        return;
    annotate_subtree(d, node_id, acc_ss, acc_ff);
    recompute_total_area(d);
}

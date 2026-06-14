#include "lp_types.hpp"

#include <algorithm>
#include <cstring>

void LpProblem::clear()
{
    branches.clear();
    ff_node_ids.clear();
    path_ids.clear();
    wns_ss_ori = tns_ss_ori = wns_ff_ori = tns_ff_ori = area_ori = 0.0;
    time_limit_sec = 600.0;
}

void LpSolution::clear()
{
    d_ss.clear();
    d_ff.clear();
    status = 0;
    solver_name.clear();
}

void lp_problem_init(LpProblem *pb)
{
    pb->clear();
}

void lp_problem_free(LpProblem *pb)
{
    pb->clear();
}

double lp_eval_branch_delay_ss(const PdDesign * /*d*/, const PdCell *c, int fanout)
{
    fanout = std::max(1, std::min(fanout, c->max_fanout));
    return c->ss_delay[fanout - 1];
}

double lp_eval_branch_delay_ff(const PdDesign * /*d*/, const PdCell *c, int fanout)
{
    fanout = std::max(1, std::min(fanout, c->max_fanout));
    return c->ff_delay[fanout - 1];
}

void lp_cell_delay_bounds(const PdDesign *d, int /*cell_idx*/, int fanout, double *ss_min,
                          double *ss_max, double *ff_min, double *ff_max)
{
    double ss_lo = 1e30, ss_hi = 0.0, ff_lo = 1e30, ff_hi = 0.0;
    for (int i = 0; i < d->n_cells; i++) {
        const double s = lp_eval_branch_delay_ss(d, &d->cells[i], fanout);
        const double f = lp_eval_branch_delay_ff(d, &d->cells[i], fanout);
        ss_lo = std::min(ss_lo, s);
        ss_hi = std::max(ss_hi, s);
        ff_lo = std::min(ff_lo, f);
        ff_hi = std::max(ff_hi, f);
    }
    *ss_min = ss_lo;
    *ss_max = ss_hi;
    *ff_min = ff_lo;
    *ff_max = ff_hi;
}

static void add_branch(LpProblem *pb, int parent, int child, LpBranchKind kind, PdDesign *d)
{
    const PdNode *cn = &d->nodes[child];
    const PdNode *pn = &d->nodes[parent];
    const int fanout = pn->nchildren > 0 ? pn->nchildren : 1;

    LpBranch b;
    b.parent_node = parent;
    b.child_node = child;
    b.kind = kind;
    b.fanout = fanout;

    if (kind == LpBranchKind::ExistingBuf) {
        b.cell_idx = cn->cell_idx;
        lp_cell_delay_bounds(d, b.cell_idx, b.fanout, &b.d_ss_min, &b.d_ss_max, &b.d_ff_min,
                             &b.d_ff_max);
        if (cn->cell_idx >= 0) {
            const PdCell *c = &d->cells[cn->cell_idx];
            b.area_per_ss = c->width * c->height;
        }
    } else {
        b.d_ss_min = 0.0;
        b.d_ff_min = 0.0;
        b.d_ss_max = 0.0;
        b.d_ff_max = 0.0;
        for (int i = 0; i < d->n_cells; i++) {
            const double s = lp_eval_branch_delay_ss(d, &d->cells[i], 1);
            const double f = lp_eval_branch_delay_ff(d, &d->cells[i], 1);
            b.d_ss_max = std::max(b.d_ss_max, s);
            b.d_ff_max = std::max(b.d_ff_max, f);
        }
        b.area_per_ss = d->cells[0].width * d->cells[0].height;
    }
    pb->branches.push_back(b);
}

int lp_build_from_design(LpProblem *pb, PdDesign *d, char *err, std::size_t err_sz)
{
    const double saved_time_limit = pb->time_limit_sec;
    lp_problem_free(pb);
    lp_problem_init(pb);
    pb->time_limit_sec = saved_time_limit;

    for (int i = 0; i < d->n_nodes; i++) {
        const PdNode *n = &d->nodes[i];
        for (int j = 0; j < n->nchildren; j++) {
            const int cid = n->children[j];
            const PdNode *c = &d->nodes[cid];
            if (c->kind == PD_NODE_BUF)
                add_branch(pb, i, cid, LpBranchKind::ExistingBuf, d);
            else if (c->kind == PD_NODE_FF)
                add_branch(pb, i, cid, LpBranchKind::Insertable, d);
        }
    }

    for (int i = 0; i < d->n_nodes; i++) {
        if (d->nodes[i].kind == PD_NODE_FF)
            pb->ff_node_ids.push_back(i);
    }
    for (int i = 0; i < d->n_paths; i++)
        pb->path_ids.push_back(i);

    pd_annotate_clock(d);
    pd_compute_timing(d);
    pb->wns_ss_ori = d->wns_setup_ss;
    pb->tns_ss_ori = d->tns_setup_ss;
    pb->wns_ff_ori = d->wns_hold_ff;
    pb->tns_ff_ori = d->tns_hold_ff;
    pb->area_ori = d->total_area;

    if (pb->branches.empty()) {
        if (err && err_sz > 0)
            std::snprintf(err, err_sz, "no branches in clock tree");
        return -1;
    }
    return 0;
}

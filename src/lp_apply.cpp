#include "lp_apply.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

static int find_best_cell_ss_ff(const PdDesign *d, double target_ss, double target_ff, int fanout)
{
    int best = 0;
    double best_err = 1e30;
    for (int i = 0; i < d->n_cells; i++) {
        const double s = lp_eval_branch_delay_ss(d, &d->cells[i], fanout);
        const double f = lp_eval_branch_delay_ff(d, &d->cells[i], fanout);
        const double err = (s - target_ss) * (s - target_ss) + (f - target_ff) * (f - target_ff);
        if (err < best_err) {
            best_err = err;
            best = i;
        }
    }
    return best;
}

int lp_apply_solution(PdDesign *d, const LpProblem *pb, const LpSolution *sol, char * /*err*/,
                      std::size_t /*err_sz*/)
{
    const int n = static_cast<int>(pb->branches.size());
    for (int b = 0; b < n; b++) {
        const LpBranch &br = pb->branches[static_cast<std::size_t>(b)];
        if (br.kind != LpBranchKind::ExistingBuf)
            continue;

        PdNode *node = &d->nodes[br.child_node];
        if (node->kind != PD_NODE_BUF)
            continue;

        node->cell_idx = find_best_cell_ss_ff(d, sol->d_ss[static_cast<std::size_t>(b)],
                                              sol->d_ff[static_cast<std::size_t>(b)], br.fanout);
        std::strncpy(node->cell, d->cells[node->cell_idx].name, PD_MAX_NAME - 1);
        node->cell[PD_MAX_NAME - 1] = '\0';
    }

    pd_annotate_clock(d);
    pd_compute_timing(d);
    return 0;
}

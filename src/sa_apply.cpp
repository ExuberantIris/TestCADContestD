#include "sa_apply.hpp"

#include <cstring>

int sa_apply_solution(PdDesign *d, const LpProblem *pb, const LpSolution *sol,
                      const LpBufferChainDp *dp_ss, const LpBufferChainDp * /*dp_ff*/,
                      char * /*err*/, std::size_t /*err_sz*/)
{
    const int n = static_cast<int>(pb->branches.size());
    for (int b = 0; b < n; b++) {
        const LpBranch &br = pb->branches[static_cast<std::size_t>(b)];
        if (br.kind != LpBranchKind::ExistingBuf)
            continue;

        const double d_ss = sol->d_ss[static_cast<std::size_t>(b)];
        const LpBufferChainEntry &ent = dp_ss->lookup(br.fanout, d_ss);
        if (!ent.reachable || ent.cell_indices.empty())
            continue;

        PdNode *node = &d->nodes[br.child_node];
        if (node->kind != PD_NODE_BUF)
            continue;

        const int ci = ent.cell_indices.back();
        node->cell_idx = ci;
        std::strncpy(node->cell, d->cells[ci].name, PD_MAX_NAME - 1);
        node->cell[PD_MAX_NAME - 1] = '\0';
    }

    pd_annotate_clock(d);
    pd_compute_timing(d);
    return 0;
}

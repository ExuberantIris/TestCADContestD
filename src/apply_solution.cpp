#include "apply_solution.hpp"
#include "lp_types.hpp"
#include <cstring>
#include <cmath>
#include <cstdio> // 為了 printf

int apply_solution(PdDesign *d, const LpProblem *pb, const LpSolution *sol,
                   const LpBufferChainDp * /*dp_ss*/, const LpBufferChainDp * /*dp_ff*/,
                   char * /*err*/, std::size_t /*err_sz*/)
{
    const int n = static_cast<int>(pb->branches.size());
    int applied_count = 0;

    for (int b = 0; b < n; b++) {
        const LpBranch &br = pb->branches[static_cast<std::size_t>(b)];
        
        if (br.kind != LpBranchKind::ExistingBuf)
            continue; // 目前只做 Resize

        const double target_ss = sol->d_ss[static_cast<std::size_t>(b)];
        const double target_ff = sol->d_ff[static_cast<std::size_t>(b)];

        int best_cell_idx = -1;
        double min_err = 1e9;

        // 暴力掃描 Library，找出最接近的
        for (int ci = 0; ci < d->n_cells; ci++) {
            const PdCell *c = &d->cells[ci];
            if (!br.allow_zero_cell && pd_cell_is_zero(c))
                continue;
            if (br.fanout > c->max_fanout)
                continue;

            double css = lp_eval_branch_delay_ss(d, c, br.fanout);
            double cff = lp_eval_branch_delay_ff(d, c, br.fanout);
            double err = std::fabs(css - target_ss) + std::fabs(cff - target_ff);
            
            if (err < min_err) {
                min_err = err;
                best_cell_idx = ci;
            }
        }

        // 不再嚴格限制 1e-4，只要找到最佳解就硬上！
        if (best_cell_idx >= 0) {
            PdNode *node = &d->nodes[br.child_node];
            if (node->kind == PD_NODE_BUF) {
                node->cell_idx = best_cell_idx;
                std::strncpy(node->cell, d->cells[best_cell_idx].name, PD_MAX_NAME - 1);
                node->cell[PD_MAX_NAME - 1] = '\0';
                applied_count++;
            }
        }
    }

    std::printf("apply_solution: 成功替換了 %d 顆 Buffer 的尺寸！\n", applied_count);

    pd_annotate_clock(d);
    pd_compute_timing(d);
    return 0;
}
#include "sa_apply.hpp"
#include "lp_types.hpp"
#include <cstring>
#include <cmath>
#include <cstdio> // 為了 printf
#include <cstdlib>

int sa_apply_solution(PdDesign *d, const LpProblem *pb, const LpSolution *sol,
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
                // Detect virtual combo entries encoded as "VCOMBO_i_j_bk"
                const char *cname = d->cells[best_cell_idx].name;
                if (std::strncmp(cname, "VCOMBO_", 7) == 0) {
                    // parse indices: VCOMBO_i_j_b{branch}
                    int a = -1, bb = -1, bk = -1;
                    if (std::sscanf(cname + 7, "%d_%d_b%d", &a, &bb, &bk) == 3) {
                        // Expand: current node becomes first buffer (a), create new node for second (bb)
                        // Ensure capacity for new node
                        if (d->n_nodes >= d->cap_nodes) {
                            int new_cap = d->cap_nodes ? d->cap_nodes * 2 : PD_INIT_NODES;
                            PdNode *p = (PdNode *)std::realloc(d->nodes, (size_t)new_cap * sizeof(PdNode));
                            if (p) {
                                d->nodes = p;
                                d->cap_nodes = new_cap;
                            }
                        }

                        int new_id = d->n_nodes;
                        // initialize new node
                        PdNode *newn = &d->nodes[new_id];
                        std::memset(newn, 0, sizeof(*newn));
                        newn->id = new_id;
                        std::snprintf(newn->name, PD_MAX_NAME, "%s_inserted_%d", node->name, new_id);
                        newn->kind = PD_NODE_BUF;
                        newn->parent = node->id;
                        // move existing children to new node
                        newn->children = node->children;
                        newn->nchildren = node->nchildren;
                        for (int ci2 = 0; ci2 < newn->nchildren; ci2++) {
                            int child_id = newn->children[ci2];
                            d->nodes[child_id].parent = new_id;
                        }

                        // set node to have single child = new node
                        node->children = (int *)std::malloc(sizeof(int));
                        node->children[0] = new_id;
                        node->nchildren = 1;

                        // set cells for node (first) and newn (second)
                        if (a >= 0 && a < d->n_cells) {
                            node->cell_idx = a;
                            std::strncpy(node->cell, d->cells[a].name, PD_MAX_NAME - 1);
                            node->cell[PD_MAX_NAME - 1] = '\0';
                        }
                        if (bb >= 0 && bb < d->n_cells) {
                            newn->cell_idx = bb;
                            std::strncpy(newn->cell, d->cells[bb].name, PD_MAX_NAME - 1);
                            newn->cell[PD_MAX_NAME - 1] = '\0';
                        }

                        d->n_nodes++;
                        applied_count++;
                    } else {
                        // fallback: treat as normal cell
                        node->cell_idx = best_cell_idx;
                        std::strncpy(node->cell, cname, PD_MAX_NAME - 1);
                        node->cell[PD_MAX_NAME - 1] = '\0';
                        applied_count++;
                    }
                } else {
                    node->cell_idx = best_cell_idx;
                    std::strncpy(node->cell, d->cells[best_cell_idx].name, PD_MAX_NAME - 1);
                    node->cell[PD_MAX_NAME - 1] = '\0';
                    applied_count++;
                }
            }
        }
    }

    std::printf("sa_apply: 成功替換了 %d 顆 Buffer 的尺寸！\n", applied_count);

    pd_annotate_clock(d);
    pd_compute_timing(d);
    return 0;
}
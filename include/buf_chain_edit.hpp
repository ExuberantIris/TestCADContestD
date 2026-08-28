#pragma once

#include <cstddef>
#include <vector>

#include "lp_buffer_dp.hpp"
#include "lp_types.hpp"

struct BranchChainState {
    int final_cell_idx = -1;
    std::vector<int> prefix_cells;
};

int buf_chain_next_new_buf_id(const PdDesign *d);

void buf_chain_capture_branch(const PdDesign *d, const LpBranch &br, BranchChainState *st);

int buf_chain_set_branch(PdDesign *d, const LpBranch &br, const BranchChainState &target,
                         int *next_new_buf_id);

const LpBufferChainEntry &buf_chain_lookup_nearest(const LpBufferChainDp *dp, int fanout,
                                                   double delay);

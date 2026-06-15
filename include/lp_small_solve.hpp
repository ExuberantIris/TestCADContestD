#pragma once

#include <cstddef>
#include <vector>

#include "lp_types.hpp"
#include "sa_eval.hpp"

/** Projected-gradient LP on a subset of branches/paths; other branches stay fixed. */
int lp_solve_subset(const LpProblem *pb, const PdDesign *d, const SaPgCtx &ctx,
                    const LpBufferChainDp *dp_ss, const LpBufferChainDp *dp_ff,
                    const std::vector<double> &fixed_ss, const std::vector<double> &fixed_ff,
                    const std::vector<int> &active_branches, const std::vector<int> &active_paths,
                    double time_limit_sec, std::vector<double> *out_ss, std::vector<double> *out_ff,
                    char *err, std::size_t err_sz);

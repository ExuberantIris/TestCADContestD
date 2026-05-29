#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "pd_c_api.hpp"

enum class LpBranchKind { ExistingBuf, Insertable };

struct LpBranch {
    int parent_node = 0;
    int child_node = 0;
    LpBranchKind kind = LpBranchKind::ExistingBuf;
    int cell_idx = -1;
    int fanout = 1;
    double d_ss_min = 0.0;
    double d_ss_max = 0.0;
    double d_ff_min = 0.0;
    double d_ff_max = 0.0;
    double area_per_ss = 0.0;
};

struct LpProblem {
    std::vector<LpBranch> branches;
    std::vector<int> ff_node_ids;
    std::vector<int> path_ids;

    double wns_ss_ori = 0.0;
    double tns_ss_ori = 0.0;
    double wns_ff_ori = 0.0;
    double tns_ff_ori = 0.0;
    double area_ori = 0.0;
    double time_limit_sec = 570.0;

    void clear();
};

struct LpSolution {
    std::vector<double> d_ss;
    std::vector<double> d_ff;
    int status = 0;
    std::string solver_name;

    void clear();
};

void lp_problem_init(LpProblem *pb);
void lp_problem_free(LpProblem *pb);
int lp_build_from_design(LpProblem *pb, PdDesign *d, char *err, std::size_t err_sz);

double lp_eval_branch_delay_ss(const PdDesign *d, const PdCell *c, int fanout);
double lp_eval_branch_delay_ff(const PdDesign *d, const PdCell *c, int fanout);
void lp_cell_delay_bounds(const PdDesign *d, int cell_idx, int fanout, double *ss_min,
                          double *ss_max, double *ff_min, double *ff_max);

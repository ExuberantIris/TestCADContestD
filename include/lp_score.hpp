#pragma once

#include <cstddef>

#include "lp_types.hpp"

struct LpMetrics {
    double wns_setup_ss = 0.0;
    double tns_setup_ss = 0.0;
    double wns_hold_ff = 0.0;
    double tns_hold_ff = 0.0;
    double area = 0.0;
    double score = 0.0;
};

void lp_compute_metrics(const PdDesign *d, LpMetrics *m);
double lp_compute_score(double wns_ss, double tns_ss, double wns_ff, double tns_ff, double area,
                        double wns_ss_ori, double tns_ss_ori, double wns_ff_ori, double tns_ff_ori,
                        double area_ori);
void lp_print_metrics(const char *label, const LpMetrics *m);

int lp_write_result_txt(const char *result_dir, const char *testcase_dir, const LpMetrics *ori,
                        const LpMetrics *opt, const char *solver_name, int solver_status,
                        double time_limit_sec, char *err, std::size_t err_sz);

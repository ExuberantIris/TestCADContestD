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

/** Competition score weights: a=TNS, b=WNS, g=Area (default 0.6, 0.2, 0.2). */
struct LpScoreWeights {
    double a = 0.6;
    double b = 0.2;
    double g = 0.2;
};

void lp_compute_metrics(const PdDesign *d, LpMetrics *m);
double lp_compute_score(double wns_ss, double tns_ss, double wns_ff, double tns_ff, double area,
                        double wns_ss_ori, double tns_ss_ori, double wns_ff_ori, double tns_ff_ori,
                        double area_ori);
/** Competition score using WNS/TNS only (no area term). */
double lp_compute_timing_score(double wns_ss, double tns_ss, double wns_ff, double tns_ff,
                               double wns_ss_ori, double tns_ss_ori, double wns_ff_ori,
                               double tns_ff_ori);
double lp_compute_weighted_score(double wns_ss, double tns_ss, double wns_ff, double tns_ff,
                                 double area, double wns_ss_ori, double tns_ss_ori,
                                 double wns_ff_ori, double tns_ff_ori, double area_ori,
                                 const LpScoreWeights &w);
void lp_print_metrics(const char *label, const LpMetrics *m);
void lp_print_timing_metrics(const char *label, const LpMetrics *m);
void lp_print_weighted_metrics(const char *label, const LpMetrics *m, const LpScoreWeights &wt);

struct LpPhaseResult {
    const char *phase_name = nullptr;
    const char *solver_name = nullptr;
    int solver_status = 0;
    double elapsed_sec = 0.0;
    long long iterations = 0;
    LpMetrics metrics{};
};

int lp_write_result_txt(const char *result_dir, const char *testcase_dir, const LpMetrics *ori,
                        const LpMetrics *opt, const char *solver_name, int solver_status,
                        double time_limit_sec, double sa_phase_limit_sec, double lp_init_sec,
                        int lp_init_ok, double sa_elapsed_sec, double wall_elapsed_sec,
                        long long sa_iterations, int use_second_best, const char *result_basename,
                        char *err, std::size_t err_sz);

/** Write baseline + multiple optimization phases into one result.txt. */
int lp_write_phased_result_txt(const char *result_dir, const char *testcase_dir,
                               const LpMetrics *ori, const LpPhaseResult *phases, int n_phases,
                               const LpScoreWeights *weights, int no_improve_limit,
                               double wall_elapsed_sec, char *err, std::size_t err_sz);

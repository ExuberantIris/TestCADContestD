#include "lp_score.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>

static double score_pos_wns(double wns)
{
    return (wns < 0.0) ? -wns : 0.0;
}

static double score_pos_tns(double tns)
{
    return (tns < 0.0) ? -tns : 0.0;
}

static double score_term_ratio(double num, double den)
{
    if (den < 1e-9) {
        if (num < 1e-9)
            return 1.0;
        return 0.0;
    }
    return num / den;
}

void lp_compute_metrics(const PdDesign *d, LpMetrics *m)
{
    pd_compute_timing(const_cast<PdDesign *>(d));
    m->wns_setup_ss = d->wns_setup_ss;
    m->tns_setup_ss = d->tns_setup_ss;
    m->wns_hold_ff = d->wns_hold_ff;
    m->tns_hold_ff = d->tns_hold_ff;
    m->area = d->total_area;
    m->score = 0.0;
}

double lp_compute_score(double wns_ss, double tns_ss, double wns_ff, double tns_ff, double area,
                        double wns_ss_ori, double tns_ss_ori, double wns_ff_ori, double tns_ff_ori,
                        double area_ori)
{
    double s = 0.0;
    const double tns_ss_p = score_pos_tns(tns_ss);
    const double wns_ss_p = score_pos_wns(wns_ss);
    const double tns_ff_p = score_pos_tns(tns_ff);
    const double wns_ff_p = score_pos_wns(wns_ff);

    s += 1.0 - score_term_ratio(tns_ss_p, score_pos_tns(tns_ss_ori));
    s += 1.0 - score_term_ratio(wns_ss_p, score_pos_wns(wns_ss_ori));
    s += 1.0 - score_term_ratio(tns_ff_p, score_pos_tns(tns_ff_ori));
    s += 1.0 - score_term_ratio(wns_ff_p, score_pos_wns(wns_ff_ori));
    s += 1.0 - score_term_ratio(area, area_ori);
    return s;
}

double lp_compute_timing_score(double wns_ss, double tns_ss, double wns_ff, double tns_ff,
                               double wns_ss_ori, double tns_ss_ori, double wns_ff_ori,
                               double tns_ff_ori)
{
    double s = 0.0;
    const double tns_ss_p = score_pos_tns(tns_ss);
    const double wns_ss_p = score_pos_wns(wns_ss);
    const double tns_ff_p = score_pos_tns(tns_ff);
    const double wns_ff_p = score_pos_wns(wns_ff);

    s += 1.0 - score_term_ratio(tns_ss_p, score_pos_tns(tns_ss_ori));
    s += 1.0 - score_term_ratio(wns_ss_p, score_pos_wns(wns_ss_ori));
    s += 1.0 - score_term_ratio(tns_ff_p, score_pos_tns(tns_ff_ori));
    s += 1.0 - score_term_ratio(wns_ff_p, score_pos_wns(wns_ff_ori));
    return s;
}

double lp_compute_weighted_score(double wns_ss, double tns_ss, double wns_ff, double tns_ff,
                                 double area, double wns_ss_ori, double tns_ss_ori,
                                 double wns_ff_ori, double tns_ff_ori, double area_ori,
                                 const LpScoreWeights &wt)
{
    const double tns_ss_p = score_pos_tns(tns_ss);
    const double wns_ss_p = score_pos_wns(wns_ss);
    const double tns_ff_p = score_pos_tns(tns_ff);
    const double wns_ff_p = score_pos_wns(wns_ff);

    const double tns_ss_term = 1.0 - score_term_ratio(tns_ss_p, score_pos_tns(tns_ss_ori));
    const double wns_ss_term = 1.0 - score_term_ratio(wns_ss_p, score_pos_wns(wns_ss_ori));
    const double tns_ff_term = 1.0 - score_term_ratio(tns_ff_p, score_pos_tns(tns_ff_ori));
    const double wns_ff_term = 1.0 - score_term_ratio(wns_ff_p, score_pos_wns(wns_ff_ori));
    const double area_term = 1.0 - score_term_ratio(area, area_ori);

    const double ss_part = (tns_ss_term + wns_ss_term);
    const double ff_part = (tns_ff_term + wns_ff_term);
    return wt.a * ss_part + wt.b * ff_part + wt.g * area_term;
}

void lp_print_weighted_metrics(const char *label, const LpMetrics *m, const LpScoreWeights &wt)
{
    std::printf("=== %s ===\n", label);
    std::printf("SS setup WNS : %.6f  TNS : %.6f\n", m->wns_setup_ss, m->tns_setup_ss);
    std::printf("FF hold  WNS : %.6f  TNS : %.6f\n", m->wns_hold_ff, m->tns_hold_ff);
    std::printf("Total area   : %.6f\n", m->area);
    std::printf("Score (a=%.4f b=%.4f g=%.4f): %.6f\n", wt.a, wt.b, wt.g, m->score);
}

void lp_print_metrics(const char *label, const LpMetrics *m)
{
    std::printf("=== %s ===\n", label);
    std::printf("SS setup WNS : %.6f  TNS : %.6f\n", m->wns_setup_ss, m->tns_setup_ss);
    std::printf("FF hold  WNS : %.6f  TNS : %.6f\n", m->wns_hold_ff, m->tns_hold_ff);
    std::printf("Total area   : %.6f\n", m->area);
    std::printf("Score (a=b=g=1): %.6f\n", m->score);
}

void lp_print_timing_metrics(const char *label, const LpMetrics *m)
{
    std::printf("=== %s ===\n", label);
    std::printf("SS setup WNS : %.6f  TNS : %.6f\n", m->wns_setup_ss, m->tns_setup_ss);
    std::printf("FF hold  WNS : %.6f  TNS : %.6f\n", m->wns_hold_ff, m->tns_hold_ff);
    std::printf("Score (timing only, no area): %.6f\n", m->score);
}

int lp_write_result_txt(const char *result_dir, const char *testcase_dir, const LpMetrics *ori,
                        const LpMetrics *opt, const char *solver_name, int solver_status,
                        double time_limit_sec, double sa_phase_limit_sec, double lp_init_sec,
                        int lp_init_ok, double sa_elapsed_sec, double wall_elapsed_sec,
                        long long sa_iterations, int use_second_best, const char *result_basename,
                        char *err, std::size_t err_sz)
{
    char path[1024];
    FILE *fp;
    const char *basename = (result_basename && result_basename[0]) ? result_basename : "result.txt";

    if (pd_join_path(path, sizeof(path), result_dir, basename) != 0) {
        if (err && err_sz > 0)
            std::snprintf(err, err_sz, "result path too long");
        return -1;
    }
    fp = std::fopen(path, "w");
    if (!fp) {
        if (err && err_sz > 0)
            std::snprintf(err, err_sz, "cannot open %s for write", path);
        return -1;
    }

    std::fprintf(fp, "sa_solver_prime result\n");
    std::fprintf(fp, "testcase_dir: %s\n", testcase_dir);
    std::fprintf(fp, "time_limit_sec: %.1f\n", time_limit_sec);
    std::fprintf(fp, "sa_phase_limit_sec: %.1f\n", sa_phase_limit_sec);
    std::fprintf(fp, "lp_init_sec: %.3f\n", lp_init_sec);
    std::fprintf(fp, "lp_init_ok: %d\n", lp_init_ok);
    std::fprintf(fp, "wall_elapsed_sec: %.3f\n", wall_elapsed_sec);
    std::fprintf(fp, "sa_elapsed_sec: %.3f\n", sa_elapsed_sec);
    std::fprintf(fp, "sa_iterations: %lld\n", static_cast<long long>(sa_iterations));
    std::fprintf(fp, "use_second_best: %d\n", use_second_best);
    std::fprintf(fp, "solver: %s\n", solver_name ? solver_name : "(unknown)");
    std::fprintf(fp, "solver_status: %d\n\n", solver_status);

    std::fprintf(fp, "=== baseline (ori) ===\n");
    std::fprintf(fp, "SS setup WNS : %.6f  TNS : %.6f\n", ori->wns_setup_ss, ori->tns_setup_ss);
    std::fprintf(fp, "FF hold  WNS : %.6f  TNS : %.6f\n", ori->wns_hold_ff, ori->tns_hold_ff);
    std::fprintf(fp, "Total area   : %.6f\n", ori->area);
    std::fprintf(fp, "Score (a=b=g=1): %.6f\n\n", ori->score);

    std::fprintf(fp, "=== after optimize ===\n");
    std::fprintf(fp, "SS setup WNS : %.6f  TNS : %.6f\n", opt->wns_setup_ss, opt->tns_setup_ss);
    std::fprintf(fp, "FF hold  WNS : %.6f  TNS : %.6f\n", opt->wns_hold_ff, opt->tns_hold_ff);
    std::fprintf(fp, "Total area   : %.6f\n", opt->area);
    std::fprintf(fp, "Score (a=b=g=1): %.6f\n", opt->score);

    std::fclose(fp);
    return 0;
}

int lp_write_phased_result_txt(const char *result_dir, const char *testcase_dir,
                               const LpMetrics *ori, const LpPhaseResult *phases, int n_phases,
                               const LpScoreWeights *weights, int no_improve_limit,
                               double wall_elapsed_sec, char *err, std::size_t err_sz)
{
    char path[1024];
    if (!result_dir || !testcase_dir || !ori || !phases || n_phases <= 0) {
        if (err && err_sz > 0)
            std::snprintf(err, err_sz, "invalid phased result arguments");
        return -1;
    }
    if (pd_join_path(path, sizeof(path), result_dir, "result.txt") != 0) {
        if (err && err_sz > 0)
            std::snprintf(err, err_sz, "result path too long");
        return -1;
    }

    FILE *fp = std::fopen(path, "w");
    if (!fp) {
        if (err && err_sz > 0)
            std::snprintf(err, err_sz, "cannot open %s for write", path);
        return -1;
    }

    std::fprintf(fp, "sa_solver result (multi-phase)\n");
    std::fprintf(fp, "testcase_dir: %s\n", testcase_dir);
    std::fprintf(fp, "wall_elapsed_sec: %.3f\n", wall_elapsed_sec);
    if (weights)
        std::fprintf(fp, "score_weights: a=%.6f b=%.6f g=%.6f\n", weights->a, weights->b,
                     weights->g);
    std::fprintf(fp, "sa_no_improve_limit: %d\n\n", no_improve_limit);

    std::fprintf(fp, "=== baseline (ori) ===\n");
    std::fprintf(fp, "SS setup WNS : %.6f  TNS : %.6f\n", ori->wns_setup_ss, ori->tns_setup_ss);
    std::fprintf(fp, "FF hold  WNS : %.6f  TNS : %.6f\n", ori->wns_hold_ff, ori->tns_hold_ff);
    std::fprintf(fp, "Total area   : %.6f\n", ori->area);
    if (weights)
        std::fprintf(fp, "Score (a=%.4f b=%.4f g=%.4f): %.6f\n\n", weights->a, weights->b,
                     weights->g, ori->score);
    else
        std::fprintf(fp, "Score: %.6f\n\n", ori->score);

    for (int i = 0; i < n_phases; i++) {
        const LpPhaseResult &ph = phases[i];
        std::fprintf(fp, "=== %s ===\n", ph.phase_name ? ph.phase_name : "phase");
        std::fprintf(fp, "solver: %s\n", ph.solver_name ? ph.solver_name : "(unknown)");
        std::fprintf(fp, "solver_status: %d\n", ph.solver_status);
        std::fprintf(fp, "elapsed_sec: %.3f\n", ph.elapsed_sec);
        std::fprintf(fp, "iterations: %lld\n", static_cast<long long>(ph.iterations));
        std::fprintf(fp, "SS setup WNS : %.6f  TNS : %.6f\n", ph.metrics.wns_setup_ss,
                     ph.metrics.tns_setup_ss);
        std::fprintf(fp, "FF hold  WNS : %.6f  TNS : %.6f\n", ph.metrics.wns_hold_ff,
                     ph.metrics.tns_hold_ff);
        std::fprintf(fp, "Total area   : %.6f\n", ph.metrics.area);
        if (weights)
            std::fprintf(fp, "Score (a=%.4f b=%.4f g=%.4f): %.6f\n", weights->a, weights->b,
                         weights->g, ph.metrics.score);
        else
            std::fprintf(fp, "Score: %.6f\n", ph.metrics.score);
        if (i + 1 < n_phases)
            std::fprintf(fp, "\n");
    }

    std::fclose(fp);
    std::printf("Wrote %s\n", path);
    return 0;
}

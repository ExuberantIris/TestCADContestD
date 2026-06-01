#include "pd_timing.h"

#include <stdio.h>

void pd_compute_timing(PdDesign *d)
{
    int i;
    int has_setup = 0;
    int has_hold = 0;

    d->wns_setup_ss = 1e30;
    d->tns_setup_ss = 0.0;
    d->wns_hold_ff = 1e30;
    d->tns_hold_ff = 0.0;

    for (i = 0; i < d->n_paths; i++) {
        PdPath *p = &d->paths[i];
        const PdNode *launch = (p->launch_id >= 0) ? &d->nodes[p->launch_id] : NULL;
        const PdNode *capture = (p->capture_id >= 0) ? &d->nodes[p->capture_id] : NULL;
        double skew_ss, skew_ff;
        double slack_setup, slack_hold;

        if (!launch || !capture)
            continue;

        skew_ss = capture->d_clk_ss - launch->d_clk_ss;
        skew_ff = capture->d_clk_ff - launch->d_clk_ff;

        slack_setup = d->clock_period - d->t_setup - p->data_ss + skew_ss;
        slack_hold = p->data_ff - d->t_hold - skew_ff;

        p->slack_setup_ss = slack_setup;
        p->slack_hold_ff = slack_hold;

        if (slack_setup < d->wns_setup_ss)
            d->wns_setup_ss = slack_setup;
        if (slack_setup < 0.0)
            d->tns_setup_ss += slack_setup;

        if (slack_hold < d->wns_hold_ff)
            d->wns_hold_ff = slack_hold;
        if (slack_hold < 0.0)
            d->tns_hold_ff += slack_hold;

        has_setup = 1;
        has_hold = 1;
    }

    if (!has_setup) {
        d->wns_setup_ss = 0.0;
        d->tns_setup_ss = 0.0;
    }
    if (!has_hold) {
        d->wns_hold_ff = 0.0;
        d->tns_hold_ff = 0.0;
    }
}

void pd_print_timing_summary(const PdDesign *d)
{
    printf("=== Phase 0 timing summary ===\n");
    printf("Clock period : %.6f\n", d->clock_period);
    printf("T_setup      : %.6f\n", d->t_setup);
    printf("T_hold       : %.6f\n", d->t_hold);
    printf("Total area   : %.6f\n", d->total_area);
    printf("SS setup WNS : %.6f  TNS : %.6f\n", d->wns_setup_ss, d->tns_setup_ss);
    printf("FF hold  WNS : %.6f  TNS : %.6f\n", d->wns_hold_ff, d->tns_hold_ff);
    printf("Paths        : %d\n", d->n_paths);
}

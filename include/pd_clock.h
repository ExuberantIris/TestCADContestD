#ifndef PD_CLOCK_H
#define PD_CLOCK_H

#include "pd_types.h"

void pd_annotate_clock(PdDesign *d);

/** Re-annotate clock delays from node_id down (after a buffer resize). */
void pd_annotate_clock_subtree(PdDesign *d, int node_id);

#endif

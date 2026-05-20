#include "pd_checker.h"
#include "pd_clock.h"
#include "pd_output.h"
#include "pd_parser.h"
#include "pd_timing.h"
#include "pd_types.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(int argc, char **argv)
{
    PdDesign design;
    char err[512];

    if (argc != 3) {
        fprintf(stderr, "Usage: %s <testcase_dir> <output_structure_path>\n", argv[0]);
        return 1;
    }

    memset(&design, 0, sizeof(design));

    if (pd_load_design(argv[1], &design, err, sizeof(err)) != 0) {
        fprintf(stderr, "Load failed: %s\n", err);
        return 1;
    }

    pd_annotate_clock(&design);
    pd_compute_timing(&design);
    pd_print_timing_summary(&design);

    if (pd_check_legality(&design, err, sizeof(err)) != 0) {
        fprintf(stderr, "Legality check failed: %s\n", err);
        pd_free_design(&design);
        return 1;
    }
    fprintf(stderr, "Legality check: %s\n", err);

    if (pd_write_structure(&design, argv[2], err, sizeof(err)) != 0) {
        fprintf(stderr, "Write failed: %s\n", err);
        pd_free_design(&design);
        return 1;
    }

    fprintf(stderr, "Wrote %s\n", argv[2]);
    pd_free_design(&design);
    return 0;
}

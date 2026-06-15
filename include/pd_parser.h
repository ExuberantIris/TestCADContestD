#ifndef PD_PARSER_H
#define PD_PARSER_H

#include <stddef.h>

#include "pd_types.h"

int pd_load_design(const char *testcase_dir, PdDesign *d, char *err, size_t err_sz);
int pd_load_design_with_structure(const char *testcase_dir, const char *structure_path, PdDesign *d,
                                  char *err, size_t err_sz);
void pd_free_design(PdDesign *d);

#endif

#ifndef PD_OUTPUT_H
#define PD_OUTPUT_H

#include <stddef.h>

#include "pd_types.h"

int pd_write_structure(const PdDesign *d, const char *out_path, char *err, size_t err_sz);

#endif

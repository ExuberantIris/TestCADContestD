#ifndef PD_CHECKER_H
#define PD_CHECKER_H

#include <stddef.h>

#include "pd_types.h"

int pd_check_legality(const PdDesign *d, char *err, size_t err_sz);

#endif

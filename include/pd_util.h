#ifndef PD_UTIL_H
#define PD_UTIL_H

#include <stddef.h>

#include "pd_types.h"

char *pd_trim(char *s);
int pd_streq(const char *a, const char *b);
int pd_join_path(char *out, size_t out_sz, const char *dir, const char *file);
int pd_find_node_by_name(const PdDesign *d, const char *name);

/** True if `name` is exactly "NEW_BUF_<non-negative integer>" with nothing else trailing - the
 *  naming rule the contest imposes on every buffer we insert. */
int pd_is_new_buf_name(const char *name);

#endif

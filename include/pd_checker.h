#ifndef PD_CHECKER_H
#define PD_CHECKER_H

#include <stddef.h>

#include "pd_types.h"

/** Verifies that `mod` is a legal evolution of `orig` under the contest's rules:
 *    - structural self-consistency (parent/children agree, no dangling output pins, every
 *      buffer's fanout within its assigned cell's max_fanout, FF nodes are sinks)
 *    - every component present in `orig` still exists in `mod` with the same name and kind
 *      (no deletion/rename)
 *    - every component in `mod` not present in `orig` is a buffer named NEW_BUF_<n>
 *    - all names in `mod` are globally unique
 *    - the relative ancestor/descendant order of every pair of original components is
 *      unchanged (new buffers may be spliced into an edge, but may not reorder anything)
 *  Returns 0 if legal, -1 and an explanatory message in `err` otherwise. */
int pd_check_legality(const PdDesign *orig, const PdDesign *mod, char *err, size_t err_sz);

#endif

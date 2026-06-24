#ifndef PD_MUTATE_H
#define PD_MUTATE_H

#include <stddef.h>

#include "pd_types.h"

/** Splits buffer node `buf_node_id` into two consecutive buffers: the existing node keeps its
 *  position and gets a single child, a freshly-inserted node named NEW_BUF_<n> (assigned
 *  `new_cell_idx`) that takes over driving every child the original node used to drive
 *  directly. Existing component names/relationships are otherwise untouched, satisfying the
 *  contest's "may only insert buffers or resize" + "existing order unchanged" constraints.
 *
 *  On success returns 0 and, if out_new_node_id is non-NULL, writes the new node's id there.
 *  Caller is responsible for re-running pd_annotate_clock/pd_compute_timing afterward. */
int pd_split_buffer(PdDesign *d, int buf_node_id, int new_cell_idx, int *out_new_node_id,
                    char *err, size_t err_sz);

/** Inserts a new buffer (named NEW_BUF_<n>, assigned `new_cell_idx`) between `parent_id` and
 *  one specific direct child `child_id` - every *other* child of parent_id is left completely
 *  untouched. Unlike pd_split_buffer (which moves every child down a level uniformly), this is
 *  what actually decouples `child_id`'s downstream skew from its siblings: they no longer
 *  share a single delay value, since only child_id's path now includes the new buffer.
 *
 *  On success returns 0 and, if out_new_node_id is non-NULL, writes the new node's id there.
 *  Caller is responsible for re-running pd_annotate_clock/pd_compute_timing afterward. */
int pd_insert_buffer_on_child(PdDesign *d, int parent_id, int child_id, int new_cell_idx,
                              int *out_new_node_id, char *err, size_t err_sz);

#endif

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

/** Removes the inverse of pd_insert_buffer_on_child: splices single-child buffer
 *  `buf_node_id` out of the tree, reconnecting its parent directly to its one child (parent's
 *  child *count* is unchanged - only that one slot's target changes) and shifting the child's
 *  whole subtree up one level. The removed node is left in `d->nodes[]` (unreachable from the
 *  root - pd_write_structure walks from the root, so it simply vanishes from output) rather
 *  than compacted out, since nothing else holds node ids stable across a compaction.
 *
 *  Only ever call this on a NEW_BUF_* node (never an original component) and only once its
 *  assigned cell is the synthetic zero-area/zero-delay cell from pd_add_zero_cell - at that
 *  point removing it changes nothing about timing or area (it was already contributing
 *  exactly zero to both), so this is purely a structural/legality cleanup.
 *
 *  On success returns 0. Caller is responsible for re-running
 *  pd_annotate_clock/pd_compute_timing afterward. */
int pd_remove_buffer(PdDesign *d, int buf_node_id, char *err, size_t err_sz);

/** Appends a synthetic "free" cell (zero area, zero SS/FF delay at every fanout) to the
 *  design's cell library. Assigning it to a NEW_BUF_* buffer and then physically removing that
 *  buffer via pd_remove_buffer leaves timing and area byte-identical to never having inserted
 *  it in the first place - this lets the resize search continuously explore "should this
 *  decoupling buffer even exist?" alongside ordinary resizing, instead of needing a separate
 *  insert/remove decision pass. Must never be selectable by anything other than a NEW_BUF_*
 *  branch (see LpBranch::allow_zero_cell) - an *existing* component resizing down to it would
 *  be electrically deleting a component the contest rules forbid removing.
 *
 *  On success returns 0 and, if out_cell_idx is non-NULL, writes the new cell's index there.
 *  Returns -1 if the library is already full (PD_MAX_CELLS). */
int pd_add_zero_cell(PdDesign *d, int *out_cell_idx, char *err, size_t err_sz);

/** True if `c` is the synthetic zero-area cell added by pd_add_zero_cell (identified by zero
 *  area, since every real standard cell occupies positive area). */
int pd_cell_is_zero(const PdCell *c);

#endif

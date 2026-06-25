#pragma once

#include <cstddef>

#include "pd_c_api.hpp"

/** Walks up from `ff_id` toward the root looking for the *nearest* ancestor that drives more
 *  than one child directly - the true branch point this FF's downstream clock latency needs
 *  decoupling from (resizing any buffer strictly between that branch point and `ff_id` is
 *  already an exclusive, uncontested knob for this FF alone, since nothing shares it; there is
 *  nothing to decouple until a real fork is reached). On success writes the branch point to
 *  `*out_ancestor` and the one of its direct children that leads down to `ff_id` to
 *  `*out_child`, and returns true. Returns false if no such branch point exists below the root
 *  (the entire chain from `ff_id` up is single-child) - notably this also covers the case
 *  where the branch point *would* be the root itself, since pd_insert_buffer_on_child requires
 *  a PD_NODE_BUF parent and the root is never one. */
bool pd_find_branch_point(const PdDesign *d, int ff_id, int *out_ancestor, int *out_child);

/** Scans the design's *current* timing (caller must have already run pd_annotate_clock +
 *  pd_compute_timing) for violating setup/hold paths, and for each violating path's launch or
 *  capture FF, locates its nearest branch point via pd_find_branch_point and inserts a
 *  dedicated buffer on that one edge via pd_insert_buffer_on_child - decoupling that FF's
 *  downstream skew from whichever sibling subtree it currently shares a delay with, without
 *  touching any other sibling. (A direct-parent-shares-children edge, the only case the
 *  previous version of this function handled, is simply the special case where the branch
 *  point is one level up - this strictly subsumes it.) Seeds each new buffer with the
 *  library's smallest-area cell (a near-"ghost" default; resize will grow it if warranted, or
 *  shrink it onto the synthetic zero cell and have it physically removed in Phase 3 if not -
 *  see pd_add_zero_cell/pd_remove_buffer). Re-annotates/re-times the design before returning.
 *
 *  Done once, before lp_build_from_design runs, so every downstream stage (LP-init bisection,
 *  incremental greedy, order-randomization, dual-seed safety net) treats the newly inserted
 *  buffers as perfectly ordinary existing buffers - zero changes needed there.
 *
 *  Returns the number of buffers actually inserted (0 if none looked beneficial), or -1 on
 *  error (see err). */
int insert_decoupling_buffers(PdDesign *d, char *err, std::size_t err_sz);

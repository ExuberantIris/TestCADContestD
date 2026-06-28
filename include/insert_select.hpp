#pragma once

#include <cstddef>

#include "pd_c_api.hpp"

/** Returns `ff_id`'s direct parent as the insertion point: `*out_ancestor` = the parent,
 *  `*out_child` = `ff_id` itself. Deliberately does *not* walk further up looking for the
 *  nearest fork - a node strictly between a fork and `ff_id` only ever sits on an exclusive,
 *  single-child chain (nothing else shares it), so a new buffer spliced in anywhere along that
 *  chain has an identical effect on `ff_id`'s achievable delay range; there is no timing
 *  advantage to walking further, only extra bookkeeping. Returns false if `ff_id` has no parent
 *  or its parent isn't a real buffer (e.g. the root drives `ff_id` directly) - matching
 *  pd_insert_buffer_on_child's own requirement that the parent be a PD_NODE_BUF. Same `ff_id`
 *  always yields the same (ancestor, child) pair, which is what lets insert_decoupling_buffers'
 *  dedup work regardless of how many violating paths reference it. */
bool pd_find_branch_point(const PdDesign *d, int ff_id, int *out_ancestor, int *out_child);

/** Scans the design's *current* timing (caller must have already run pd_annotate_clock +
 *  pd_compute_timing) for violating setup/hold paths, and for each violating path's launch or
 *  capture FF, locates its insertion edge via pd_find_branch_point and inserts a dedicated
 *  buffer on that one edge via pd_insert_buffer_on_child - decoupling that FF's downstream skew
 *  from whichever sibling subtree it currently shares a delay with, without touching any other
 *  sibling. The same FF can be the launch/capture endpoint of several violating paths (or be
 *  picked up via both lookups for the same path); since pd_find_branch_point is a pure function
 *  of `ff_id`, every one of those lookups resolves to the exact same (ancestor, child) edge.
 *  Candidate edges are collected into a std::set before any insertion happens, so duplicates
 *  are dropped and each edge is only ever spliced once, however many times it was found.
 *  Seeds each new buffer with the library's smallest-area cell (a near-"ghost" default; resize
 *  will grow it if warranted, or shrink it onto the synthetic zero cell and have it physically
 *  removed in Phase 3 if not - see pd_add_zero_cell/pd_remove_buffer). Re-annotates/re-times the
 *  design before returning.
 *
 *  Done once, before lp_build_from_design runs, so every downstream stage (LP-init bisection,
 *  incremental greedy, order-randomization, dual-seed safety net) treats the newly inserted
 *  buffers as perfectly ordinary existing buffers - zero changes needed there.
 *
 *  Returns the number of buffers actually inserted (0 if none looked beneficial), or -1 on
 *  error (see err). */
int insert_decoupling_buffers(PdDesign *d, char *err, std::size_t err_sz);

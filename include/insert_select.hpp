#pragma once

#include <cstddef>

#include "pd_c_api.hpp"

/** Scans the design's *current* timing (caller must have already run pd_annotate_clock +
 *  pd_compute_timing) for violating setup/hold paths, and for each violating path's launch or
 *  capture FF whose direct parent buffer drives more than one child (i.e. shares its delay
 *  with siblings), inserts a dedicated buffer on just that one edge via
 *  pd_insert_buffer_on_child - decoupling that FF's skew from its siblings without touching
 *  them. Seeds each new buffer with the library's smallest-area cell (a near-"ghost" default;
 *  resize will grow it if warranted or keep shrinking it toward the cheapest option if not -
 *  no separate "remove" path is ever needed). Re-annotates/re-times the design before
 *  returning.
 *
 *  Done once, before lp_build_from_design runs, so every downstream stage (LP-init bisection,
 *  incremental greedy, order-randomization, dual-seed safety net) treats the newly inserted
 *  buffers as perfectly ordinary existing buffers - zero changes needed there.
 *
 *  Returns the number of buffers actually inserted (0 if none looked beneficial), or -1 on
 *  error (see err). */
int insert_decoupling_buffers(PdDesign *d, char *err, std::size_t err_sz);

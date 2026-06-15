#pragma once

#include "lp_score.hpp"

#include <cstddef>

/** Path to the solver parameter file (relative to ProblemD project root). */
inline constexpr const char *kSaParamsPath = "src/sa_params.txt";

/** Tunable solver / SA parameters loaded from src/sa_params.txt. */
struct SaParams {
    /** Wall-clock budget for the whole solver run (seconds). */
    double total_time_limit_sec = 600.0;
    /** Wall-clock budget for phase-2 SA (seconds). */
    double sa_time_limit_sec = 180.0;

    /** Stop after this many consecutive SA iterations with no best-score improvement. */
    int no_improve_limit = 10000;
    /** Batch SA move count per batch (0 = Metropolis hybrid mode). */
    int sa_batch_size = 20;

    /** Gap-refine: pick from top-K branches by target gap. */
    int sa_leaf_ff_pick_count = 40;
    /** Per-branch moves without improvement before blacklisting. */
    int sa_branch_no_improve_limit = 1000;

    /** Metropolis initial temperature. */
    double sa_temperature_init = 1.0;
    /** Metropolis multiplicative cooling per iteration. */
    double sa_temperature_decay = 0.9995;
    /** Reset temperature when it drops below this value. */
    double sa_temperature_floor = 1e-4;

    /** Phase-2: probability of path-directed move when violations exist. */
    double phase2_path_move_prob = 0.85;
    /** Phase-2: pick violating path from top-N worst (weighted slack). */
    int phase2_top_path_pool = 20;
    /** Phase-2: pick area-recovery branch from top-N highest delay. */
    int phase2_area_branch_pool = 24;

    /** Extra margin added to max branch delay when building DP tables. */
    double dp_delay_margin = 0.02;

    LpScoreWeights score_weights{};
};

/** Load src/sa_params.txt. Missing keys keep struct defaults. */
void sa_params_load(SaParams *out, const char *path);

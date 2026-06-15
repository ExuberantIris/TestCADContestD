#pragma once

#include "lp_score.hpp"

#include <cstddef>

/** Tunable SA / score parameters loaded from text/sa_params.txt. */
struct SaParams {
    int no_improve_limit = 1000;
    LpScoreWeights score_weights{};
};

/** Load sa_params.txt (see text/sa_params.txt). Missing keys keep defaults. */
void sa_params_load(SaParams *out, const char *path);

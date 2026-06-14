#pragma once

#include <cstddef>

#include "lp_types.hpp"

/** Dump multi-objective LP (mo_projected_gradient) inputs for inspection. */
int lp_write_mo_input_dump(const LpProblem *pb, const PdDesign *d, const char *out_dir,
                           char *err, std::size_t err_sz);

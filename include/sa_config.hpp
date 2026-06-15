#pragma once

/** Centralized segment-tree SA parameters. */
namespace SaConfig {

/** Wall-clock budget for SA phase (seconds). */
inline constexpr double kSaTimeLimitSec = 180.0;

/** Number of branches with largest target-vs-current gap picked per SA iteration (K). */
inline constexpr int kSaLeafFfPickCount = 40;

/** Consecutive SA iterations without timing-score improvement before stopping (N). */
inline constexpr int kSaNoImproveLimit = 1000;

/** Per-branch moves without timing-score improvement before blacklisting. */
inline constexpr int kSaBranchNoImproveLimit = 1000;

/** Metropolis temperature initial value. */
inline constexpr double kSaTemperatureInit = 1.0;

/** Multiplicative cooling per SA iteration. */
inline constexpr double kSaTemperatureDecay = 0.9995;

/** Reset temperature to init when it drops below this value. */
inline constexpr double kSaTemperatureFloor = 1e-4;

} // namespace SaConfig

#pragma once

/** Centralized small-LP SA tunable parameters. */
namespace SaConfig {

/** Number of worst paths selected per SA cycle (K). */
inline constexpr int kSmallLpPathCount = 40;

/** Stop after this many consecutive cycles without score improvement (N). */
inline constexpr int kSmallLpNoImproveLimit = 1000;

/** Wall-clock budget per local LP solve (seconds). */
inline constexpr double kSmallLpTimeLimitSec = 0.1;

} // namespace SaConfig

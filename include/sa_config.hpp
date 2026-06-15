#pragma once

/** Centralized SA tunable parameters. */
namespace SaConfig {

/** Consecutive SA iterations without objective improvement before stopping (T). */
inline constexpr int kNoImproveLimit = 100000;

/** Per-branch assignments without objective improvement before blacklisting (N). */
inline constexpr int kBranchNoImproveLimit = 1000;

/** Initial top-path pool size for SA path selection. */
inline constexpr int kTopPathPoolInit = 20;

/** Increase top-path pool size every this many SA iterations. */
inline constexpr int kTopPathPoolGrowEvery = 500;

/** How much to increase top-path pool size each growth step. */
inline constexpr int kTopPathPoolGrowStep = 10;

/** First Q SA iterations optimize setup TNS only (ignore hold/WNS/area/score). */
inline constexpr int kSetupTnsOnlyIters = 100000;

} // namespace SaConfig

#pragma once
#include <Eigen/Core>
#include "depth_filter_util/type.h"

namespace ORB_SLAM3 {
namespace seed {
enum SeedType{
    NotSeed,
    Seed,
    ConvergedSeed,
    alreadyChangeToPoint
};

enum SeedStateIndex {
    kMu,
    kSigma2,
    kA,
    kB,
};

// -----------------------------------------------------------------------------
// Accessors

inline double mu(const Eigen::Ref<const SeedState>& mu_sigma2_a_b) {
    return mu_sigma2_a_b(0);
}

inline double sigma2(const Eigen::Ref<const SeedState>& mu_sigma2_a_b) {
    return mu_sigma2_a_b(1);
}

inline double a(const Eigen::Ref<const SeedState>& mu_sigma2_a_b) {
    return mu_sigma2_a_b(2);
}

inline double b(const Eigen::Ref<const SeedState>& mu_sigma2_a_b) {
    return mu_sigma2_a_b(3);
}

// -----------------------------------------------------------------------------
// Inverse Depth Parametrization

inline double getDepth(const Eigen::Ref<const SeedState>& mu_sigma2_a_b) {
    return 1.0 / mu_sigma2_a_b(0);
}

inline double getInvDepth(const Eigen::Ref<const SeedState>& mu_sigma2_a_b) {
    return mu_sigma2_a_b(0);
}

inline double getInvMinDepth(
    const Eigen::Ref<const SeedState>& mu_sigma2_a_b) {
    return mu_sigma2_a_b(0) + 3 * std::sqrt(mu_sigma2_a_b(1));
}

inline double getInvMaxDepth(
    const Eigen::Ref<const SeedState>& mu_sigma2_a_b) {
    return std::max(mu_sigma2_a_b(0) - 3 * std::sqrt(mu_sigma2_a_b(1)), 0.00000001);
}

inline double getMeanFromDepth(double depth) { return 1.0 / depth; }

inline double getMeanRangeFromDepthMinMax(double depth_min,
                                             double /*depth_max*/) {
    return 1.0 / depth_min;
}

inline double getInitSigma2FromMuRange(double mu_range) {
    return mu_range * mu_range / 36.0;
}

inline bool isConverged(const Eigen::Ref<const SeedState>& mu_sigma2_a_b,
                        double mu_range,
                        double sigma2_convergence_threshold) {
    // If initial uncertainty was reduced by factor sigma2_convergence_threshold
    // we accept the seed as converged.
    const double thresh = mu_range / sigma2_convergence_threshold;
    return (mu_sigma2_a_b(1) < thresh * thresh);
}

inline double getSigma2FromDepthSigma(double depth,
                                         double depth_sigma) {
    const double sigma =
        0.5 * (1.0 / std::max(0.000000000001, depth - depth_sigma) -
               1.0 / (depth + depth_sigma));
    return sigma * sigma;
}

// -----------------------------------------------------------------------------
// Utils

inline void increaseOutlierProbability(Eigen::Ref<SeedState>& mu_sigma2_a_b) {
    mu_sigma2_a_b(3) += 1;
}

}  // namespace seed
}  // namespace ORB_SLAM3

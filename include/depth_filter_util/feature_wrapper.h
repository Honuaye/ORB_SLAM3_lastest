#pragma once

#include <memory>
#include <Eigen/Core>
#include "SVOmatcher.h"
#include "depth_filter.h"
#include "depth_filter_util/type.h"

namespace ORB_SLAM3 {
// struct SeedRef {
//     FramePtr keyframe;
//     int seed_id = -1;
//     SeedRef(const FramePtr& _keyframe, const int _seed_id)
//         : keyframe(_keyframe), seed_id(_seed_id) {
//         ;
//     }
//     SeedRef() = default;
//     ~SeedRef() = default;
// };

/** @todo (MWE) */
class FeatureWrapper {
 public:
    // EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    FeatureWrapper() {
        px = Keypoint::Zero();
        f = BearingVector::Zero();
        level = 1;
        score = 0;
    }
    FeatureWrapper(
                   Keypoint _px,
                   BearingVector _f,
                   int _pyramid_level,
                   double _score
                   )
        :
          px(_px),
          f(_f),
          level(_pyramid_level),
          score(_score)
    {}

    // FeatureType& type;            //!< Type can be corner or edgelet.
    Keypoint px;      //!< Coordinates in pixels on pyramid level 0.
    // Eigen::Ref<Keypoint> px;      //!< Coordinates in pixels on pyramid level 0.
    BearingVector f;  //!< Unit-bearing vector of the feature.
    // Eigen::Ref<GradientVector> grad;  //!< Dominant gradient direction for edglets, normalized.
    double score;
    int level;  //!< Image pyramid level where feature was extracted.
    // PointPtr& landmark;
    // SeedRef& seed_ref;
    // int& track_id;
};

}  // namespace ORB_SLAM3

#pragma once

#include <Eigen/Core>
#include <sophus/se3.hpp>
#include "feature_wrapper.h"
#include "depth_filter_util/type.h"

namespace ORB_SLAM3 {

// Forward declarations.
class FeatureWrapper;
class GeometricCamera;

namespace warp {

void getWarpMatrixAffine(
                         GeometricCamera* cam_ref,
                         GeometricCamera* cam_cur,
                        //  const GeometricCamera& cam_ref,
                        //  const GeometricCamera& cam_cur,
                         const Eigen::Matrix<double, 2, 1>& px_ref,
                         const Eigen::Matrix<double, 3, 1>& f_ref,
                         const double depth_ref,
                         const Sophus::SE3<double>& T_cur_ref,
                         const int level_ref,
                         Eigen::Matrix2d* A_cur_ref);

int getBestSearchLevel(const Eigen::Matrix2d& A_cur_ref,
                       const int max_level);

bool warpAffine(const Eigen::Matrix2d& A_cur_ref,
                const cv::Mat& img_ref,
                const Keypoint& px_ref,
                // const Eigen::Ref<Keypoint>& px_ref,
                const int level_ref,
                const int level_cur,
                const int halfpatch_size,
                uint8_t* patch);
}  // namespace warp
}  // namespace ORB_SLAM3

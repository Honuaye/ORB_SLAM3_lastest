#pragma once
#include <array>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <thread>
#include "depth_filter_util/patch_score.h"
#include "depth_filter.h"
#include "depth_filter_util/feature_wrapper.h"
#include "depth_filter_util/type.h"
// #include <svo/common/types.h>

namespace ORB_SLAM3 {

/// Subpixel refinement of a reference feature patch with the current image.
/// Implements the inverse-compositional approach (see "Lucas-Kanade 20 Years
/// on"
/// paper by Baker.
namespace feature_alignment {

bool align2D(const cv::Mat& cur_img,
             uint8_t* ref_patch_with_border,
             uint8_t* ref_patch,
             const int n_iter,
             const bool affine_est_offset,
             const bool affine_est_gain,
             Keypoint& cur_px_estimate,
             bool no_simd = false,
             std::vector<Eigen::Vector2f>* each_step = nullptr);


}  // namespace feature_alignment
}  // namespace ORB_SLAM3

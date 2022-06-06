#include <cstdlib>
#include <random>
#include <chrono>
#include "depth_filter_util/SVOmatcher.h"
#include "depth_filter_util/feature_wrapper.h"
#include "depth_filter_util/feature_alignment.h"
#include "depth_filter.h"
#include "depth_filter_util/patch_warp.h"


namespace ORB_SLAM3 {

SVOMatcher::MatchResult SVOMatcher::findEpipolarMatchDirect(
    const KeyFrame& ref_frame,
    const Frame& cur_frame,
    const Transformation& T_cur_ref,
    const FeatureWrapper& ref_ftr,
    const double d_estimate_inv,
    const double d_min_inv,
    const double d_max_inv,
    double& depth) {
    int zmssd_best = PatchScore::threshold();
    // // Compute start and end of epipolar line in old_kf for match search, on
    // // image plane
    const Eigen::Matrix<double, 3, 1> A = T_cur_ref.rotationMatrix() * ref_ftr.f +
                            T_cur_ref.translation() * d_min_inv;
    const Eigen::Matrix<double, 3, 1> B = T_cur_ref.rotationMatrix() * ref_ftr.f +
                            T_cur_ref.translation() * d_max_inv;
    // const Eigen::Matrix<double, 3, 1> A = T_cur_ref.getRotation().rotate(ref_ftr.f) +
    //                         T_cur_ref.getPosition() * d_min_inv;
    // const Eigen::Matrix<double, 3, 1> B = T_cur_ref.getRotation().rotate(ref_ftr.f) +
    //                         T_cur_ref.getPosition() * d_max_inv;
    Eigen::Vector2d px_A, px_B;
    // cur_frame.cam()->project3(A, &px_A);
    // cur_frame.cam()->project3(B, &px_B);
    px_A = cur_frame.mpCamera->project(A);
    px_B = cur_frame.mpCamera->project(B);
    epi_image_ = px_A - px_B;
    // Compute affine warp matrix
    warp::getWarpMatrixAffine(ref_frame.mpCamera, cur_frame.mpCamera, ref_ftr.px,
                              ref_ftr.f,
                              1.0 / std::max(0.000001, d_estimate_inv),
                              T_cur_ref, ref_ftr.level, &A_cur_ref_);
    // feature pre-selection
    reject_ = false;
    // prepare for match
    //    - find best search level
    //    - warp the reference patch
    int max_level = 3;
    search_level_ = warp::getBestSearchLevel(A_cur_ref_, max_level);
    // search_level_ = warp::getBestSearchLevel(A_cur_ref_,
        // ref_frame.mpORBextractorLeft->mvImagePyramid.size() - 1);
    // length and direction on SEARCH LEVEL
    epi_length_pyramid_ = epi_image_.norm() / (1 << search_level_);
    GradientVector epi_dir_image = epi_image_.normalized();
    if(!ref_frame.mframe_) return MatchResult::kFailWarp;
    if (!warp::warpAffine(A_cur_ref_, ref_frame.mframe_->mpORBextractorLeft->mvImagePyramid[ref_ftr.level],
                          ref_ftr.px, ref_ftr.level, search_level_,
                          kHalfPatchSize + 1, patch_with_border_)) {
        if(log_print_) {
            printf("MatchResult::kFailWarp\t ");
        }
        return MatchResult::kFailWarp;
    }
    createPatchFromPatchWithBorder(patch_with_border_, kPatchSize, patch_);
    // Case 1: direct search locally if the epipolar line is too short
    if (epi_length_pyramid_ < 2.0) {
        px_cur_ = (px_A + px_B) / 2.0;
        MatchResult res =
            findLocalMatch(cur_frame, epi_dir_image, search_level_, px_cur_);
        if (res != MatchResult::kSuccess) return res;

        // cur_frame.cam()->backProject3(px_cur_, &f_cur_);
        cv::Point2f px_cur_cv(px_cur_(0), px_cur_(1));
        f_cur_ = cur_frame.mpCamera->unprojectEig(px_cur_cv).cast<double>();
        f_cur_.normalize();
        if(depth_filter_utils::depthFromTriangulation(T_cur_ref, ref_ftr.f,
                                                     f_cur_, &depth)) {
            return MatchResult::kSuccess;
        } else {
            if(log_print_) {
                printf("MatchResult::kFailTriangulation\t ");
            }
            return MatchResult::kFailTriangulation;
        }
    }
    // Case 2: search along the epipolar line for the best match
    PatchScore patch_score(patch_);  // precompute for reference patch
    // Eigen::Matrix<double, 3, 1> C = T_cur_ref.getRotation().rotate(ref_ftr.f) +
    //                   T_cur_ref.getPosition() * d_estimate_inv;
    const Eigen::Matrix<double, 3, 1> C = T_cur_ref.rotationMatrix() * ref_ftr.f +
                            T_cur_ref.translation() * d_estimate_inv;
    scanEpipolarLine(cur_frame, A, B, C, patch_score, search_level_, &px_cur_,
                     &zmssd_best);
    // check if the best match is good enough
    if (zmssd_best < PatchScore::threshold()) {
        if (options_.subpix_refinement) {
            MatchResult res = findLocalMatch(cur_frame, epi_dir_image,
                                             search_level_, px_cur_);
            if (res != MatchResult::kSuccess) {
                if(log_print_) {
                    printf("MatchResult::kFailfindLocalMatch\t ");
                }
                return res;
            }
        }
        // cur_frame.cam()->backProject3(px_cur_, &f_cur_);
        cv::Point2f px_cur_cv(px_cur_(0), px_cur_(1));
        f_cur_ = cur_frame.mpCamera->unprojectEig(px_cur_cv).cast<double>();
        f_cur_.normalize();
        if(depth_filter_utils::depthFromTriangulation(T_cur_ref, ref_ftr.f,
                                                     f_cur_, &depth)) {
            return MatchResult::kSuccess;
        } else {
            return MatchResult::kFailTriangulation;
        }
    } else {
        if(log_print_) {
            printf("MatchResult::kFailScore \t ");
        }
        return MatchResult::kFailScore;
    }
}

SVOMatcher::MatchResult SVOMatcher::findLocalMatch(
    const Frame& frame,
    const Eigen::Ref<GradientVector>& direction,
    const int patch_level,
    Keypoint& px_cur) {
    Keypoint px_scaled(px_cur / (1 << patch_level));
    bool res;
    res = feature_alignment::align2D(
        frame.mpORBextractorLeft->mvImagePyramid[patch_level], patch_with_border_, patch_,
        options_.align_max_iter, options_.affine_est_offset_,
        options_.affine_est_gain_, px_scaled);

    if (!res) return MatchResult::kFailAlignment;
    px_cur = px_scaled * (1 << patch_level);
    return MatchResult::kSuccess;
}

bool SVOMatcher::updateZMSSD(const Frame& frame,
                          const Eigen::Vector2i& pxi,
                          const int patch_level,
                          const PatchScore& patch_score,
                          int* zmssd_best) {
    // // TODO interpolation would probably be a good idea
    // // frame
    uint8_t* cur_patch_ptr =
        frame.mpORBextractorLeft->mvImagePyramid[patch_level].data +
        // frame.img_pyr_[patch_level].data +
        (pxi[1] - kHalfPatchSize) * frame.mpORBextractorLeft->mvImagePyramid[patch_level].step +
        // (pxi[1] - kHalfPatchSize) * frame.img_pyr_[patch_level].step +
        (pxi[0] - kHalfPatchSize);
    int zmssd = patch_score.computeScore(cur_patch_ptr,
                                         frame.mpORBextractorLeft->mvImagePyramid[patch_level].step);
                                        //  frame.img_pyr_[patch_level].step);
    if (zmssd < *zmssd_best) {
        *zmssd_best = zmssd;
        return true;
    } else
        return false;
}

bool SVOMatcher::isPatchWithinImage(const Frame& frame,
                                 const Eigen::Vector2i& pxi,
                                 const int patch_level) {
    return !(
        pxi[0] < kPatchSize || pxi[1] < kPatchSize ||
        pxi[0] >=
            (static_cast<int>(frame.mpORBextractorLeft->mvImagePyramid[0].cols / (1 << patch_level)) -
            // (static_cast<int>(frame.cam()->imageWidth() / (1 << patch_level)) -
             kPatchSize) ||
        pxi[1] >=
            (static_cast<int>(frame.mpORBextractorLeft->mvImagePyramid[0].cols / (1 << patch_level)) -
            // (static_cast<int>(frame.cam()->imageHeight() / (1 << patch_level)) -
             kPatchSize));
}

void SVOMatcher::scanEpipolarLine(const Frame& frame,
                               const Eigen::Vector3d& A,
                               const Eigen::Vector3d& B,
                               const Eigen::Vector3d& C,
                               const PatchScore& patch_score,
                               const int patch_level,
                               Keypoint* image_best,
                               int* zmssd_best) {
    scanEpipolarUnitPlane(frame, A, B, C, patch_score, patch_level,
                            image_best, zmssd_best);
    // TODO-YHH 
    // if (options_.scan_on_unit_sphere)
    //     scanEpipolarUnitSphere(frame, A, B, C, patch_score, patch_level,
    //                            image_best, zmssd_best);
    // else
    //     scanEpipolarUnitPlane(frame, A, B, C, patch_score, patch_level,
    //                           image_best, zmssd_best);
}

void SVOMatcher::scanEpipolarUnitPlane(const Frame& frame,
                                    const Eigen::Vector3d& A,
                                    const Eigen::Vector3d& B,
                                    const Eigen::Vector3d& C,
                                    const PatchScore& patch_score,
                                    const int patch_level,
                                    Keypoint* image_best,
                                    int* zmssd_best) {
    // if there're too many steps, we only search for a limited range around the
    // center
    //    while keeping the step size small enough to check each pixel on the
    //    image plane
    size_t n_steps = epi_length_pyramid_ / 0.7;  // one step per pixel
    Eigen::Vector2d step = (project2(A) - project2(B)) / n_steps;
    if (n_steps > options_.max_epi_search_steps) {
        /* TODO
        printf("WARNING: skip epipolar search: %d evaluations, px_lenght=%f,
        d_min=%f, d_max=%f.\n",
               n_steps, epi_length_, d_min_inv, d_max_inv);
        */
        n_steps = options_.max_epi_search_steps;
    }
    // now we sample along the epipolar line
    Eigen::Vector2d uv_C = project2(C);
    Eigen::Vector2d uv = uv_C;
    Eigen::Vector2d uv_best = uv;
    bool forward = true;
    Eigen::Vector2i last_checked_pxi(0, 0);
    for (size_t i = 0; i < n_steps; ++i, uv += step) {
        Eigen::Vector2d px;
        px = frame.mpCamera->project(unproject2d(uv));
        // frame.cam()->project3(unproject2d(uv), &px);
        Eigen::Vector2i pxi(
            px[0] / (1 << patch_level) + 0.5,
            px[1] / (1 << patch_level) + 0.5);  // +0.5 to round to closest int
        if (pxi == last_checked_pxi) continue;
        last_checked_pxi = pxi;
        // check if the patch is full within the new frame
        if (!isPatchWithinImage(frame, pxi, patch_level)) {
            // change search direction if pixel is out of field of view
            if (forward) {
                // reverse search direction
                i = n_steps * 0.5;
                step = -step;
                uv = uv_C;
                forward = false;
                continue;
            } else
                break;
        }
        if (updateZMSSD(frame, pxi, patch_level, patch_score, zmssd_best))
            uv_best = uv;
        if (forward && i > n_steps * 0.5) {
            // reverse search direction
            step = -step;
            uv = uv_C;
            forward = false;
        }
    }
    // convert uv_best to image coordinates
    Eigen::Vector2d projected;
    projected = frame.mpCamera->project(unproject2d(uv_best));
    // frame.cam()->project3(unproject2d(uv_best), &projected);
    *image_best = projected.cast<double>();
}


}  // namespace ORB_SLAM3

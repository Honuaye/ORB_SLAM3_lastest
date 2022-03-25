

#include "depth_filter_util/feature_alignment.h"

#ifdef __SSE2__
#include <emmintrin.h>
#endif
#ifdef __ARM_NEON__
#include <arm_neon.h>
#endif

#include <glog/logging.h>
#include <Eigen/Dense>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// #include <svo/direct/patch_utils.h>

namespace ORB_SLAM3 {
namespace feature_alignment {

#define SUBPIX_VERBOSE 0
#define SVO_DISPLAY_ALIGN_1D 0

bool align2D(const cv::Mat& cur_img,
             uint8_t* ref_patch_with_border,
             uint8_t* ref_patch,
             const int n_iter,
             const bool affine_est_offset,
             const bool affine_est_gain,
             Keypoint& cur_px_estimate,
             bool no_simd,
             std::vector<Eigen::Vector2f>* each_step) {
#ifdef __ARM_NEON__
    if (!no_simd)
        return align2D_NEON(cur_img, ref_patch_with_border, ref_patch, n_iter,
                            cur_px_estimate);
#endif

    if (each_step) each_step->clear();

    const int halfpatch_size_ = 4;
    const int patch_size_ = 8;
    const int patch_area_ = 64;
    bool converged = false;

    // We optimize feature position and two affine parameters.
    // compute derivative of template and prepare inverse compositional
    float __attribute__((__aligned__(16))) ref_patch_dx[patch_area_];
    float __attribute__((__aligned__(16))) ref_patch_dy[patch_area_];
    Eigen::Matrix4f H;
    H.setZero();

    // compute gradient and hessian
    const int ref_step = patch_size_ + 2;
    float* it_dx = ref_patch_dx;
    float* it_dy = ref_patch_dy;
    for (int y = 0; y < patch_size_; ++y) {
        uint8_t* it = ref_patch_with_border + (y + 1) * ref_step + 1;
        for (int x = 0; x < patch_size_; ++x, ++it, ++it_dx, ++it_dy) {
            Eigen::Vector4f J;
            J[0] = 0.5 * (it[1] - it[-1]);
            J[1] = 0.5 * (it[ref_step] - it[-ref_step]);

            // If not using the affine compensation, force the jacobian to be
            // zero.
            J[2] = affine_est_offset ? 1.0 : 0.0;
            J[3] = affine_est_gain ? -1.0 * it[0] : 0.0;

            *it_dx = J[0];
            *it_dy = J[1];
            H += J * J.transpose();
        }
    }
    // If not use affine compensation, force update to be zero by
    // * setting the affine parameter block in H to identity
    // * setting the residual block to zero (see below)
    if (!affine_est_offset) {
        H(2, 2) = 1.0;
    }
    if (!affine_est_gain) {
        H(3, 3) = 1.0;
    }
    Eigen::Matrix4f Hinv = H.inverse();
    float mean_diff = 0;
    float alpha = 1.0;

    // Compute pixel location in new image:
    float u = cur_px_estimate.x();
    float v = cur_px_estimate.y();

    if (each_step) each_step->push_back(Eigen::Vector2f(u, v));

    // termination condition
    const float min_update_squared =
        0.03 *
        0.03;  // TODO I suppose this depends on the size of the image (ate)
    const int cur_step = cur_img.step.p[0];
    // float chi2 = 0;
    Eigen::Vector4f update;
    update.setZero();
    for (int iter = 0; iter < n_iter; ++iter) {
        int u_r = std::floor(u);
        int v_r = std::floor(v);
        if (u_r < halfpatch_size_ || v_r < halfpatch_size_ ||
            u_r >= cur_img.cols - halfpatch_size_ ||
            v_r >= cur_img.rows - halfpatch_size_)
            break;

        if (std::isnan(u) || std::isnan(v))  // TODO very rarely this can
                                             // happen, maybe H is singular?
                                             // should not be at corner.. check
            return false;

        // compute interpolation weights
        float subpix_x = u - u_r;
        float subpix_y = v - v_r;
        float wTL = (1.0 - subpix_x) * (1.0 - subpix_y);
        float wTR = subpix_x * (1.0 - subpix_y);
        float wBL = (1.0 - subpix_x) * subpix_y;
        float wBR = subpix_x * subpix_y;

        // loop through search_patch, interpolate
        uint8_t* it_ref = ref_patch;
        float* it_ref_dx = ref_patch_dx;
        float* it_ref_dy = ref_patch_dy;
        // float new_chi2 = 0.0;
        Eigen::Vector4f Jres;
        Jres.setZero();
        for (int y = 0; y < patch_size_; ++y) {
            uint8_t* it = (uint8_t*)cur_img.data +
                          (v_r + y - halfpatch_size_) * cur_step + u_r -
                          halfpatch_size_;
            for (int x = 0; x < patch_size_;
                 ++x, ++it, ++it_ref, ++it_ref_dx, ++it_ref_dy) {
                float search_pixel = wTL * it[0] + wTR * it[1] +
                                     wBL * it[cur_step] +
                                     wBR * it[cur_step + 1];
                float res = search_pixel - alpha * (*it_ref) + mean_diff;
                Jres[0] -= res * (*it_ref_dx);
                Jres[1] -= res * (*it_ref_dy);

                // If affine compensation is used,
                // set Jres with respect to affine parameters.
                if (affine_est_offset) {
                    Jres[2] -= res;
                }

                if (affine_est_gain) {
                    Jres[3] -= (-1) * res * (*it_ref);
                }
                // new_chi2 += res*res;
            }
        }
        // If not use affine compensation, force update to be zero.
        if (!affine_est_offset) {
            Jres[2] = 0.0;
        }
        if (!affine_est_gain) {
            Jres[3] = 0.0;
        }
        /*
        if(iter > 0 && new_chi2 > chi2)
        {
    #if SUBPIX_VERBOSE
          cout << "error increased." << endl;
    #endif
          u -= update[0];
          v -= update[1];
          break;
        }
        chi2 = new_chi2;
    */
        update = Hinv * Jres;
        u += update[0];
        v += update[1];
        mean_diff += update[2];
        alpha += update[3];

        if (each_step) each_step->push_back(Eigen::Vector2f(u, v));

#if SUBPIX_VERBOSE
        cout << "Iter " << iter << ":"
             << "\t u=" << u << ", v=" << v << "\t update = " << update[0]
             << ", " << update[1] << "\t new chi2 = " << new_chi2 << endl;
#endif

        if (update[0] * update[0] + update[1] * update[1] <
            min_update_squared) {
#if SUBPIX_VERBOSE
            cout << "converged." << endl;
#endif
            converged = true;
            break;
        }
    }

    cur_px_estimate << u, v;
    (void)no_simd;

    return converged;
}

}  // namespace feature_alignment
}  // namespace ORB_SLAM3

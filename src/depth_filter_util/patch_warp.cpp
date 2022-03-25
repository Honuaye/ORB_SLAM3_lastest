#include <sophus/se3.hpp>
#include "depth_filter_util/patch_warp.h"
#include "depth_filter_util/type.h"
#include "GeometricCamera.h"

namespace ORB_SLAM3 {
class GeometricCamera;
namespace warp {

// TODO(cfo) take inverse depth!
// 计算两帧之间的仿射变换，用窗口的3个点做重投影变换
void getWarpMatrixAffine(GeometricCamera* cam_ref,
                         GeometricCamera* cam_cur,
                         const Eigen::Matrix<double, 2, 1>& px_ref,
                         const Eigen::Matrix<double, 3, 1>& f_ref,
                         const double depth_ref,
                         const Sophus::SE3<double>& T_cur_ref,
                         const int level_ref,
                         Eigen::Matrix2d* A_cur_ref) {
    // CHECK_NOTNULL(A_cur_ref);
    // Compute affine warp matrix A_ref_cur
    const int kHalfPatchSize = 5;
    const Eigen::Vector3d xyz_ref = f_ref * depth_ref;
    Eigen::Vector3d xyz_du_ref, xyz_dv_ref;
    // NOTE: project3 has no guarantee that the returned vector is unit length
    // - for pinhole: z component is 1 (unit plane)
    // - for omnicam: norm is 1 (unit sphere)
    Eigen::Vector2d point1 =
        px_ref + Eigen::Vector2d(kHalfPatchSize, 0) * (1 << level_ref);
    cv::Point2f p2D1(point1(0), point1(1));
    xyz_du_ref = cam_ref->unprojectEig(p2D1).cast<double>();
    Eigen::Vector2d point2 =
        px_ref + Eigen::Vector2d(0, kHalfPatchSize) * (1 << level_ref);
    cv::Point2f p2D2(point2(0), point2(1));
    xyz_dv_ref = cam_ref->unprojectEig(p2D2).cast<double>();
    xyz_du_ref *= xyz_ref[2];
    xyz_dv_ref *= xyz_ref[2];
    Eigen::Matrix<double, 2, 1> px_cur, px_du_cur, px_dv_cur;
    px_cur = cam_cur->project(T_cur_ref * xyz_ref);
    px_du_cur = cam_cur->project(T_cur_ref * xyz_du_ref);
    px_dv_cur = cam_cur->project(T_cur_ref * xyz_dv_ref);
    A_cur_ref->col(0) = (px_du_cur - px_cur) / kHalfPatchSize;
    A_cur_ref->col(1) = (px_dv_cur - px_cur) / kHalfPatchSize;

    // cam_ref->backProject3(
    //     px_ref + Eigen::Vector2d(kHalfPatchSize, 0) * (1 << level_ref),
    //     &xyz_du_ref);
    // cam_ref->backProject3(
    //     px_ref + Eigen::Vector2d(0, kHalfPatchSize) * (1 << level_ref),
    //     &xyz_dv_ref);
    // xyz_du_ref *= xyz_ref[2];
    // xyz_dv_ref *= xyz_ref[2];
    // // if (cam_ref->getType() == Camera::Type::kPinhole) {
    // //     xyz_du_ref *= xyz_ref[2];
    // //     xyz_dv_ref *= xyz_ref[2];
    // // } else {
    // //     xyz_du_ref.normalize();
    // //     xyz_dv_ref.normalize();
    // //     xyz_du_ref *= depth_ref;
    // //     xyz_dv_ref *= depth_ref;
    // // }
    // Eigen::Matrix<double, 2, 1> px_cur, px_du_cur, px_dv_cur;
    // cam_cur->project3(T_cur_ref * xyz_ref, &px_cur);
    // cam_cur->project3(T_cur_ref * xyz_du_ref, &px_du_cur);
    // cam_cur->project3(T_cur_ref * xyz_dv_ref, &px_dv_cur);
    // A_cur_ref->col(0) = (px_du_cur - px_cur) / kHalfPatchSize;
    // A_cur_ref->col(1) = (px_dv_cur - px_cur) / kHalfPatchSize;
}

// 这个是啥理论 
// 学习参考 : https://blog.csdn.net/luoshi006/article/details/80699720
// 找到 cur 和 ref 中 patch 缩放一致的层
int getBestSearchLevel(const Eigen::Matrix2d& A_cur_ref,
                       const int max_level) {
    // Compute patch level in other image
    int search_level = 0;
    double D = A_cur_ref.determinant();
    while (D > 3.0 && search_level < max_level) {
        search_level += 1;
        D *= 0.25;
    }
    return search_level;
}

bool warpAffine(const Eigen::Matrix2d& A_cur_ref,
                const cv::Mat& img_ref,
                const Keypoint& px_ref,
                // const Eigen::Ref<Eigen::Matrix<double, 2, 1>>& px_ref,
                const int level_ref,
                const int search_level,
                const int halfpatch_size,
                uint8_t* patch) {
    Eigen::Matrix2f A_ref_cur = A_cur_ref.inverse().cast<float>() * (1 << search_level);
    // if (std::isnan(A_ref_cur(0, 0))) {
    //     LOG(WARNING)
    //         << "Affine warp is NaN, probably camera has no translation";
    //     return false;
    // }
    // Perform the warp on a larger patch.
    uint8_t* patch_ptr = patch;
    const Eigen::Vector2f px_ref_pyr = px_ref.cast<float>() / (1 << level_ref);
    const int stride = img_ref.step.p[0];
    for (int y = -halfpatch_size; y < halfpatch_size; ++y) {
        for (int x = -halfpatch_size; x < halfpatch_size; ++x, ++patch_ptr) {
            const Eigen::Vector2f px_patch(x, y);
            const Eigen::Vector2f px(A_ref_cur * px_patch + px_ref_pyr);
            const int xi = std::floor(px[0]);
            const int yi = std::floor(px[1]);
            if (xi < 0 || yi < 0 || xi + 1 >= img_ref.cols ||
                yi + 1 >= img_ref.rows)
                return false;
            else {
                const float subpix_x = px[0] - xi;
                const float subpix_y = px[1] - yi;
                const float w00 = (1.0f - subpix_x) * (1.0f - subpix_y);
                const float w01 = (1.0f - subpix_x) * subpix_y;
                const float w10 = subpix_x * (1.0f - subpix_y);
                const float w11 = 1.0f - w00 - w01 - w10;
                const uint8_t* const ptr = img_ref.data + yi * stride + xi;
                *patch_ptr =
                    static_cast<uint8_t>(w00 * ptr[0] + w01 * ptr[stride] +
                                         w10 * ptr[1] + w11 * ptr[stride + 1]);
            }
        }
    }
    return true;
}

}  // namespace warp
}  // namespace ORB_SLAM3

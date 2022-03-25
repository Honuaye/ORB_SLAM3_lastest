#pragma once
#include <array>
#include <Eigen/Core>
#include <thread>
#include "depth_filter_util/patch_score.h"
#include "depth_filter.h"
#include "depth_filter_util/feature_wrapper.h"
#include "depth_filter_util/type.h"

namespace ORB_SLAM3 {
// // forward declarations
// class Point;
class FeatureWrapper;
class GeometricCamera;
class Frame;
class KeyFrame;
class Atlas;

/// Patch-matcher for reprojection-matching and epipolar search in  triangulation.
class SVOMatcher {
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    bool log_print_ = false;
    // bool log_print_ = true;
    static const int kHalfPatchSize = 4;
    static const int kPatchSize = 8;
    typedef ORB_SLAM3::patch_score::ZMSSD<kHalfPatchSize> PatchScore;
    typedef std::shared_ptr<SVOMatcher> Ptr;
    struct Options {
        bool align_1d =
            false;  //!< in epipolar search: align patch 1D along epipolar line
        int align_max_iter = 10;  //!< number of iterations for aligning the
                                  //! feature patches in gauss newton
        double max_epi_length_optim = 2.0;  //!< max length of epipolar line to
        //! skip epipolar search and directly
        //! go to img align
        size_t max_epi_search_steps =
            100;  //!< max number of evaluations along epipolar line
        bool subpix_refinement = true;  //!< do gauss newton feature patch
                                        //! alignment after epipolar search
        bool epi_search_edgelet_filtering = true;
        bool scan_on_unit_sphere = true;
        double epi_search_edgelet_max_angle = 0.7;
        bool verbose = false;
        bool use_affine_warp_ = true;
        bool affine_est_offset_ = true;
        bool affine_est_gain_ = false;
        double max_patch_diff_ratio = 2.0;
    } options_;

    enum class MatchResult {
        kSuccess,
        kFailScore,
        kFailTriangulation,
        kFailVisibility,
        kFailWarp,
        kFailAlignment,
        kFailRange,
        kFailAngle,
        kFailCloseView,
        kFailLock,
        kFailTooFar
    };

    uint8_t patch_[kPatchSize * kPatchSize] __attribute__((aligned(16)));
    uint8_t patch_with_border_[(kPatchSize + 2) * (kPatchSize + 2)]
        __attribute__((aligned(16)));
    Eigen::Matrix2d A_cur_ref_;  //!< affine warp matrix
    Eigen::Vector2d
        epi_image_;  //!< vector from epipolar start to end on the image plane
    double epi_length_pyramid_;  //!< length of epipolar line segment in pixels
                                 //! on pyrimid level (only used for epipolar
    //! search)
    double h_inv_;  //!< hessian of 1d image alignment along epipolar line
    int search_level_;
    bool reject_;
    Keypoint px_cur_;
    Eigen::Matrix<double, 3, 1> f_cur_;
    SVOMatcher() = default;
    ~SVOMatcher() = default;

    MatchResult findEpipolarMatchDirect(const Frame& ref_frame,
                                        const Frame& cur_frame,
                                        const Transformation& T_cur_ref,
                                        const FeatureWrapper& ref_ftr,
                                        const double d_estimate_inv,
                                        const double d_min_inv,
                                        const double d_max_inv,
                                        double& depth);

    /// search epipolar line between A~C~B for the best match with respect to patch score
    /// the search is done on patch_level, returns image coordinates and best ZMSSD
    void scanEpipolarLine(const Frame& frame,
                          const Eigen::Vector3d& A,
                          const Eigen::Vector3d& B,
                          const Eigen::Vector3d& C,
                          const PatchScore& patch_score,
                          const int patch_level,
                          Keypoint* image_best,
                          int* zmssd_best);


 private:
    // TODO(zzc): perhaps some of these should be inline
    /// local optimization for patch_ and patch_with_border_ in *frame* around
    /// *px_cur*(image)
    MatchResult findLocalMatch(const Frame& frame,
                               const Eigen::Ref<GradientVector>& direction,
                               const int patch_level,
                               Keypoint& px_cur);

    /// update best zmssd, if a better one is find, return true
    bool updateZMSSD(const Frame& frame,
                     const Eigen::Vector2i& pxi,
                     const int patch_level,
                     const PatchScore& patch_score,
                     int* zmssd_best);

    /// check is patch is fully within image
    bool isPatchWithinImage(const Frame& frame,
                            const Eigen::Vector2i& pxi,
                            const int patch_level);

    /// search along the epipolar line on image plane
    /// we sample on the UNIT PLANE and check corresponding patches on the image
    /// plane
    void scanEpipolarUnitPlane(const Frame& frame,
                               const Eigen::Vector3d& A,
                               const Eigen::Vector3d& B,
                               const Eigen::Vector3d& C,
                               const PatchScore& patch_score,
                               const int patch_level,
                               Keypoint* image_best,
                               int* zmssd_best);

    /// search along the epipolar line on image plane
    /// we sample on the UNIT SPHERE and check corresponding patches on the
    /// image plane
    void scanEpipolarUnitSphere(const Frame& frame,
                                const Eigen::Vector3d& A,
                                const Eigen::Vector3d& B,
                                const Eigen::Vector3d& C,
                                const PatchScore& patch_score,
                                const int patch_level,
                                Keypoint* image_best,
                                int* zmssd_best);

    inline Eigen::Vector3d unproject2d(const Eigen::Vector2d& v) {
        return Eigen::Vector3d(v[0], v[1], 1.0);
    }
    template<typename Derived>
    Eigen::Matrix<typename Derived::Scalar, 2, 1> project2(const Eigen::MatrixBase<Derived>& v) {
        EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived, 3, 1);
        return v.template head<2>() / v(2);
    }

    inline void createPatchFromPatchWithBorder(
        const uint8_t* const patch_with_border,
        const int patch_size,
        uint8_t* patch) {
        uint8_t* patch_ptr = patch;
        for (int y = 1; y < patch_size + 1; ++y, patch_ptr += patch_size) {
            const uint8_t* ref_patch_border_ptr =
                patch_with_border + y * (patch_size + 2) + 1;
            for (int x = 0; x < patch_size; ++x)
                patch_ptr[x] = ref_patch_border_ptr[x];
        }
    }
};

}  // namespace ORB

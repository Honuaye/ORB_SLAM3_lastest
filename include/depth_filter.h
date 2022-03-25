
#pragma once

#include <queue>
#include <memory>  // std::shared_ptr
#include <mutex>
#include <thread>
#include <condition_variable>
#include <Eigen/Core>
// #include <so3.hpp>
#include <Frame.h>
#include <KeyFrame.h>
#include <depth_filter_util/seed.h>
#include <depth_filter_util/SVOmatcher.h>
#include "depth_filter_util/type.h"

namespace ORB_SLAM3 {

class GeometricCamera;
class Frame;
class KeyFrame;
class Atlas;

// // using FramePtr = std::shared_ptr<Frame>;
// using FramePtr = Frame*;
// // using KeyFramePtr = std::shared_ptr<KeyFrame>;
// using KeyFramePtr = KeyFrame*;
// // using SeedState = Eigen::Matrix<double, 4, 1>;
// using Transformation = Sophus::SE3<double>;
// // using BearingVector = Eigen::Vector3d;
// using BearingVector = Eigen::Matrix<double, 3, 1>;
// using GradientVector = Eigen::Matrix<double, 2, 1>;

// // forward declarations
// class AbstractDetector;
// typedef std::shared_ptr<AbstractDetector> DetectorPtr;
struct DetectorOptions;

/// Depth-filter config parameters
struct DepthFilterOptions {
    /// Threshold for the uncertainty of the seed. If seed's sigma2 is thresh
    /// smaller than the inital sigma, it is considered as converged.
    /// Default value is 200. If seeds should converge quicker, set it to 50 or
    /// if you want very precise 3d points, set it higher.
    // double seed_convergence_sigma2_thresh = 10;
    // double seed_convergence_sigma2_thresh = 50;
    double seed_convergence_sigma2_thresh = 200.0;

    /// Threshold for map point seeds convergence. Should be higher to make sure
    /// we have an accurate map (for loop closing).
    double mappoint_convergence_sigma2_thresh = 500.0;

    /// Use inverse-depth parametrization for seeds.
    /// Default is true. Set to false if you are using the depth-filter to
    /// reconstruct small objects with fixed depth range.
    bool use_inverse_depth = true;

    /// Specify the max pyramid level for the matcher.
    /// Normally, you don't need to change this parameters.
    size_t max_search_level = 2;

    /// Show additional debug output
    bool verbose = false;

    /// Start separate thread for seed updates
    bool use_threaded_depthfilter = true;

    /// Update the 3D point linked by the feature in the seed (false for
    /// REMODE-CPU)
    bool update_3d_point = true;

    /// Do epipolar search on unit sphere
    bool scan_epi_unit_sphere = false;

    /// Restrict number of features per frame.
    size_t max_n_seeds_per_frame = 200;

    size_t max_map_seeds_per_frame = 200;

    /// use affine model to compensate for brightness change
    bool affine_est_offset = true;
    bool affine_est_gain = false;

    ///
    bool extra_map_points = false;
};

/// Depth filter implements the Bayesian Update proposed in:
/// "Video-based, Real-Time Multi View Stereo" by G. Vogiatzis and C.
/// Hern??ndez.
/// In Image and Vision Computing, 29(7):434-441, 2011.
class DepthFilter {
 protected:
    struct Job {
        enum Type { UPDATE, SEED_INIT } type;
        FramePtr cur_frame;
        KeyFramePtr cur_frame_init;
        KeyFramePtr ref_frame;
        // size_t ref_frame_seed_index;
        double min_depth, max_depth, mean_depth;

        /// Default constructor
        Job() : cur_frame(nullptr), ref_frame(nullptr) {}

        /// Constructor for seed update
        Job(const FramePtr& _cur_frame,
            const KeyFramePtr& _ref_frame)
            : type(UPDATE),
              cur_frame_init(nullptr),
              cur_frame(_cur_frame),
              ref_frame(_ref_frame) {}
              // ref_frame_seed_index(_ref_index) {}

        /// Constructor for seed initialization
        Job(const KeyFramePtr& f, double min_d, double max_d, double mean_d)
            : type(SEED_INIT),
              cur_frame_init(f),
              cur_frame(nullptr),
              ref_frame(nullptr),
              min_depth(min_d),
              max_depth(max_d),
              mean_depth(mean_d) {}
    };

 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<DepthFilter> Ptr;
    typedef std::mutex mutex_t;
    typedef std::unique_lock<mutex_t> ulock_t;
    typedef std::queue<Job> JobQueue;

    DepthFilter(const DepthFilterOptions& options);
    virtual ~DepthFilter();
    void addKeyframe(const KeyFramePtr& frame,
                     const double depth_mean,
                     const double depth_min,
                     const double depth_max);
    size_t updateSeedsWithFrame(const std::vector<KeyFramePtr>& frames_with_seeds,
                       const FramePtr& new_frame);

    size_t updateSeedsWithKeyFrame(
      const std::vector<KeyFramePtr>& ref_frames_with_seeds, const KeyFramePtr& cur_frame,
      ORB_SLAM3::Atlas* mpAtlas);

    void reset();
    mutex_t feature_detector_mut_;
    // DetectorPtr feature_detector_;
    // DetectorPtr sec_feature_detector_;  // for extra points used for loop closing
    DepthFilterOptions options_;

 protected:
    mutex_t jobs_mut_;
    JobQueue jobs_;
    std::condition_variable jobs_condvar_;
    std::unique_ptr<std::thread> thread_;
    bool quit_thread_ = false;
    // Matcher::Ptr matcher_;
    void updateSeedsLoop();
};

namespace depth_filter_utils {

static int false_in_findEpipolarMatchDirect_;
static int false_in_check_visibility_;
static int false_in_update_;

/// Initialize new seeds from a frame.
void initializeSeeds(KeyFramePtr frame,
                    //  const DetectorPtr& feature_detector,
                     const size_t max_n_seeds,
                     const float min_depth,
                     const float max_depth,
                     const float mean_depth);

/// Update Seed
bool updateSeedWithFrame(const FramePtr& cur_frame,
                KeyFramePtr& ref_frame,
                const size_t& seed_index,
                // Matcher& matcher,
                const double sigma2_convergence_threshold,
                const bool check_visibility = true,
                const bool check_convergence = false,
                const bool use_vogiatzis_update = true);

bool updateSeedWithKeyFrame(const KeyFramePtr& cur_keyframe,
                KeyFramePtr& ref_keyframe,
                ORB_SLAM3::Atlas * mpAtlas,
                const double sigma2_convergence_threshold,
                const bool check_visibility = true,
                const bool check_convergence = false,
                const bool use_vogiatzis_update = true);

bool updateFilterVogiatzis(const double z,
                           const double tau2,
                           const double z_range,
                           Eigen::Ref<SeedState>& seed);

bool updateFilterGaussian(const double z,
                          const double tau2,
                          Eigen::Ref<SeedState>& seed);

/// Compute the uncertainty of the measurement.
double computeTau(const Transformation& T_ref_cur,
                  const BearingVector& f,
                  const double z,
                  const double px_error_angle);

bool depthFromTriangulation(const Transformation& T_search_ref,
                                            const Eigen::Vector3d& f_ref,
                                            const Eigen::Vector3d& f_cur,
                                            double* depth);

template<class T>
inline T normPdf(const T x, const T mean, const T sigma) {
  T exponent = x - mean;
  exponent *= -exponent;
  exponent /= 2 * sigma * sigma;
  T result = std::exp(exponent);
  result /= sigma * std::sqrt(2 * M_PI);
  return result;
}


}  // namespace depth_filter_utils

}  // namespace ORB_SLAM3

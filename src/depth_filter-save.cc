#include <algorithm>

#include "depth_filter.h"
#include "ORBmatcher.h"


namespace ORB_SLAM3 {

DepthFilter::DepthFilter(const DepthFilterOptions& options)
    : options_(options) {
}

DepthFilter::~DepthFilter() {
}

void DepthFilter::reset() {
}

void DepthFilter::addKeyframe(const KeyFramePtr& keyframe,
                              const double depth_mean,
                              const double depth_min,
                              const double depth_max) {
    depth_filter_utils::initializeSeeds(
        keyframe, options_.max_n_seeds_per_frame, depth_min, depth_max, depth_mean);
}

// (SVO用了10帧共视觉的关键帧)
size_t DepthFilter::updateSeedsWithFrame(
    const std::vector<KeyFramePtr>& ref_frames_with_seeds, const FramePtr& cur_frame) {
    size_t n_success = 0;
    for (auto ref_frame : ref_frames_with_seeds) {
        if(ref_frame->depth_filter_processing_) {
            depth_filter_utils::updateSeedWithFrame(cur_frame, ref_frame, 
                options_.seed_convergence_sigma2_thresh, true, false, true);
            // if (depth_filter_utils::updateSeed(*cur_frame, *ref_frame,
            //                                    i, *matcher_, cur_thresh,
            //                                    true, false)) {
            //     ++n_success;
            // }
        }
    }
    return n_success;
}

// (SVO用了10帧共视觉的关键帧)
size_t DepthFilter::updateSeedsWithKeyFrame(
    const std::vector<KeyFramePtr>& ref_frames_with_seeds, const KeyFramePtr& cur_frame,
    ORB_SLAM3::Atlas* mpAtlas) {
    size_t n_success = 0;
    for (auto ref_frame : ref_frames_with_seeds) {
        if(ref_frame->depth_filter_processing_) {
            depth_filter_utils::updateSeedWithKeyFrame(cur_frame, ref_frame, mpAtlas,
                options_.seed_convergence_sigma2_thresh, true, false, true);
        }
    }
    return n_success;
}

void DepthFilter::updateSeedsLoop() {
}

namespace depth_filter_utils {

void initializeSeeds(KeyFramePtr keyframe,
                     const size_t max_n_seeds,
                     const float depth_min,
                     const float depth_max,
                     const float depth_mean) {
    Eigen::Matrix<double, 4, 1> invmu_sigma2_a_b_init;
    invmu_sigma2_a_b_init(0, 0) = depth_mean;
    invmu_sigma2_a_b_init(1, 0) = 1.0 / depth_min;
    invmu_sigma2_a_b_init(2, 0) = 10.0;
    invmu_sigma2_a_b_init(3, 0) = 10.0;
    const auto & mvpMapPoints = keyframe->GetMapPointMatches();
    for(size_t i = 0; i < mvpMapPoints.size(); ++i) {
        if(mvpMapPoints.at(i)) {
            // keyframe->depth_filter_state_.at(i) = false;
            keyframe->depth_filter_state_.at(i) = seed::SeedType::NotSeed;
            continue;
        }
        Eigen::Matrix<double, 3, 1> f;
        auto px = keyframe->mvKeysUn.at(i);
        auto x = (px.pt.x - keyframe->cx) / keyframe->fx;
        auto y = (px.pt.y - keyframe->cy) / keyframe->fy;
        f << static_cast<double>(x), static_cast<double>(y), 1;
        f.normalize();
        keyframe->invmu_sigma2_a_b_vec_.block<4, 1>(0, i) = invmu_sigma2_a_b_init;
        keyframe->bearing_vecs_.block<3, 1>(0, i) = f;
        // keyframe->depth_filter_state_.at(i) = true;
        keyframe->depth_filter_state_.at(i) = seed::SeedType::Seed;
        keyframe->seed_mu_range_ = 1.0 / depth_min;
    }
    keyframe->depth_filter_processing_ = true;
}

bool updateSeedWithFrame(const FramePtr& cur_frame,
                KeyFramePtr& ref_keyframe,
                const double sigma2_convergence_threshold,
                const bool check_visibility,
                const bool check_convergence,
                const bool use_vogiatzis_update) {
    cur_frame->ComputeBoW();
    ORBmatcher matcher(0.8, true);
    std::vector<pair<size_t, size_t> > vMatchedPairs;
    matcher.SearchForUpdateDepthFilter(cur_frame, ref_keyframe, vMatchedPairs);
    std::chrono::steady_clock::time_point optimize_1 = std::chrono::steady_clock::now();

    for(const auto& match : vMatchedPairs) {
        auto cur_index = match.first;
        auto ref_index = match.second;
        // if(!ref_keyframe->depth_filter_state_.at(ref_index)) continue;
        if(ref_keyframe->depth_filter_state_.at(ref_index) != seed::SeedType::Seed) {
            continue;
        }
        Eigen::Ref<SeedState> state = ref_keyframe->invmu_sigma2_a_b_vec_.col(ref_index);

        // sanity checks
        if (std::isnan(seed::mu(state)) || std::isnan(std::sqrt(seed::sigma2(state)))) {
            // set bad
            // ref_keyframe->depth_filter_state_.at(ref_index) = false;
            ref_keyframe->depth_filter_state_.at(ref_index) = seed::SeedType::NotSeed;
            continue;
        }

        Transformation T_21;
        Sophus::SE3f Tc1_w = ref_keyframe->GetPose();
        Sophus::SE3f Tc2_w = cur_frame->GetPose();
        // T_21 = static_cast<Transformation>(Tc2_w.inverse() * Tc1_w);
        T_21 = (Tc2_w.inverse() * Tc1_w).cast<double>();

        Eigen::Vector3d f_cur;
        auto px = cur_frame->mvKeysUn.at(cur_index);
        auto x = (px.pt.x - cur_frame->cx) / cur_frame->fx;
        auto y = (px.pt.y - cur_frame->cy) / cur_frame->fy;
        f_cur << static_cast<double>(x), static_cast<double>(y), 1;
        f_cur.normalize();
        double depth = 0.0;
        if(!depth_filter_utils::depthFromTriangulation(T_21, ref_keyframe->bearing_vecs_.col(ref_index), f_cur, &depth)) {
            seed::increaseOutlierProbability(state);
            continue;
        }
        // // compute tau
        // // TODO
        constexpr double px_noise = 1.0;
        double px_error_angle;
        // static double px_error_angle = cur_frame.getAngleError(px_noise);
        px_error_angle = std::atan(px_noise/(2.0*cur_frame->fx)) + std::atan(px_noise/(2.0*cur_frame->fy));
        const double depth_sigma = computeTau(T_21.inverse(), ref_keyframe->bearing_vecs_.col(ref_index), depth, px_error_angle);
        // // update the estimate
        if (use_vogiatzis_update) {
            if (!updateFilterVogiatzis(
                    seed::getMeanFromDepth(depth),
                    seed::getSigma2FromDepthSigma(depth, depth_sigma),
                    ref_keyframe->seed_mu_range_, state)) {
                // set bad
                printf("updateFilterVogiatzis bad !!!!!!! \n");
                // ref_keyframe->depth_filter_state_.at(ref_index) = false;
            ref_keyframe->depth_filter_state_.at(ref_index) = seed::SeedType::NotSeed;
                // ref_ftr.type = FeatureType::kOutlier;
                continue;
            }
        } else {
            if (!updateFilterGaussian(
                    seed::getMeanFromDepth(depth),
                    seed::getSigma2FromDepthSigma(depth, depth_sigma), state)) {
                // set bad
                printf("updateFilterGaussian bad !!!!!!! \n");
                // ref_keyframe->depth_filter_state_.at(ref_index) = false;
            ref_keyframe->depth_filter_state_.at(ref_index) = seed::SeedType::NotSeed;
                // ref_ftr.type = FeatureType::kOutlier;
                continue;
            }
        }
        // check if converged
        if (seed::isConverged(state, ref_keyframe->seed_mu_range_, sigma2_convergence_threshold)) {
            // printf("isConverged  ");
            // ref_keyframe->depth_filter_state_.at(ref_index) = seed::SeedType::ConvergedSeed;
        }
        ref_keyframe->invmu_sigma2_a_b_vec_.block<4, 1>(0, ref_index) = state;
    }
    return true;
}

bool updateSeedWithKeyFrame(const KeyFramePtr& cur_keyframe,
                KeyFramePtr& ref_keyframe,
                ORB_SLAM3::Atlas* mpAtlas,
                const double sigma2_convergence_threshold,
                const bool check_visibility,
                const bool check_convergence,
                const bool use_vogiatzis_update) {
    // std::chrono::steady_clock::time_point optimize_0 = std::chrono::steady_clock::now();
    cur_keyframe->ComputeBoW();
    float th = 0.6f;
    ORBmatcher matcher(th,false);
    std::vector<pair<size_t, size_t> > vMatchedPairs;
    matcher.SearchForTriangulation(cur_keyframe,ref_keyframe,vMatchedPairs,false,false);
    size_t converged_num = 0;
    for(const auto& match : vMatchedPairs) {
        auto cur_index = match.first;
        auto ref_index = match.second;
        // if(!ref_keyframe->depth_filter_state_.at(ref_index)) continue;
        if(ref_keyframe->depth_filter_state_.at(ref_index) != seed::SeedType::Seed) {
            continue;
        }
        Eigen::Ref<SeedState> state = ref_keyframe->invmu_sigma2_a_b_vec_.col(ref_index);
        // sanity checks
        if (std::isnan(seed::mu(state)) || std::isnan(std::sqrt(seed::sigma2(state)))) {
            // set bad
            // ref_keyframe->depth_filter_state_.at(ref_index) = false;
            ref_keyframe->depth_filter_state_.at(ref_index) = seed::SeedType::NotSeed;
            continue;
        }
        Transformation T_21;
        Sophus::SE3f Tc1_w = ref_keyframe->GetPose();
        Sophus::SE3f Tc2_w = cur_keyframe->GetPose();
        // T_21 = static_cast<Transformation>(Tc2_w.inverse() * Tc1_w);
        T_21 = (Tc2_w.inverse() * Tc1_w).cast<double>();
        Eigen::Vector3d f_cur;
        auto px = cur_keyframe->mvKeysUn.at(cur_index);
        auto x = (px.pt.x - cur_keyframe->cx) / cur_keyframe->fx;
        auto y = (px.pt.y - cur_keyframe->cy) / cur_keyframe->fy;
        f_cur << static_cast<double>(x), static_cast<double>(y), 1;
        f_cur.normalize();
        double depth = 0.0;
        if(!depth_filter_utils::depthFromTriangulation(T_21, ref_keyframe->bearing_vecs_.col(ref_index), f_cur, &depth)) {
            seed::increaseOutlierProbability(state);
            continue;
        }
        // // compute tau
        // // TODO
        constexpr double px_noise = 1.0;
        double px_error_angle;
        // static double px_error_angle = cur_keyframe.getAngleError(px_noise);
        px_error_angle = std::atan(px_noise/(2.0*cur_keyframe->fx)) + std::atan(px_noise/(2.0*cur_keyframe->fy));
        const double depth_sigma = computeTau(T_21.inverse(), ref_keyframe->bearing_vecs_.col(ref_index), depth, px_error_angle);
        // // update the estimate
        if (use_vogiatzis_update) {
            if (!updateFilterVogiatzis(
                    seed::getMeanFromDepth(depth),
                    seed::getSigma2FromDepthSigma(depth, depth_sigma),
                    ref_keyframe->seed_mu_range_, state)) {
                // set bad
                printf("updateFilterVogiatzis bad !!!!!!! \n");
                // ref_keyframe->depth_filter_state_.at(ref_index) = false;
            ref_keyframe->depth_filter_state_.at(ref_index) = seed::SeedType::NotSeed;
                // ref_ftr.type = FeatureType::kOutlier;
                continue;
            }
        } else {
            if (!updateFilterGaussian(
                    seed::getMeanFromDepth(depth),
                    seed::getSigma2FromDepthSigma(depth, depth_sigma), state)) {
                // set bad
                printf("updateFilterGaussian bad !!!!!!! \n");
                // ref_keyframe->depth_filter_state_.at(ref_index) = false;
            ref_keyframe->depth_filter_state_.at(ref_index) = seed::SeedType::NotSeed;
                // ref_ftr.type = FeatureType::kOutlier;
                continue;
            }
        }
        // check if converged
        double sigma2_convergence_threshold2 = 50;
        if (seed::isConverged(state, ref_keyframe->seed_mu_range_, sigma2_convergence_threshold2)) {
            // printf("isConverged  ");
            converged_num++;
            ref_keyframe->depth_filter_state_.at(ref_index) = seed::SeedType::ConvergedSeed;
            // Triangulation is succesfull
            Eigen::Vector3d p_c = ref_keyframe->bearing_vecs_.col(ref_index) * ref_keyframe->getSeedDepth(ref_index);
            auto T_wc = ref_keyframe->GetPoseInverse();
            Eigen::Vector3f x3D = T_wc.rotationMatrix() * p_c.cast<float>() + T_wc.translation();
            // MapPoint* pMP = new MapPoint(x3D, cur_keyframe, mpAtlas->GetCurrentMap());
            // pMP->AddObservation(cur_keyframe, cur_index);
            // pMP->AddObservation(ref_keyframe, ref_index);
            // cur_keyframe->AddMapPoint(pMP, cur_index);
            // ref_keyframe->AddMapPoint(pMP, ref_index);
            // pMP->ComputeDistinctiveDescriptors();
            // pMP->UpdateNormalAndDepth();
            // mpAtlas->AddMapPoint(pMP);
        }
        // update
        ref_keyframe->invmu_sigma2_a_b_vec_.block<4, 1>(0, ref_index) = state;
    }
    std::cout 
        << "vMatchedPairs.size  = " << vMatchedPairs.size()
        << "\t converged_num = " << converged_num
        << "\n";
    return true;
}

bool updateFilterVogiatzis(const double z,  // Measurement
                           const double tau2,
                           const double mu_range,
                           Eigen::Ref<SeedState>& mu_sigma2_a_b) {
    double& mu = mu_sigma2_a_b(0);
    double& sigma2 = mu_sigma2_a_b(1);
    double& a = mu_sigma2_a_b(2);
    double& b = mu_sigma2_a_b(3);

    const double norm_scale = std::sqrt(sigma2 + tau2);
    if (std::isnan(norm_scale)) {
        std::cout << "Update Seed: Sigma2+Tau2 is NaN \n";
        return false;
    }

    const double oldsigma2 = sigma2;
    const double s2 = 1.0 / (1.0 / sigma2 + 1.0 / tau2);
    const double m = s2 * (mu / sigma2 + z / tau2);
    const double uniform_x = 1.0 / mu_range;
    // double C1
    double C1 = a / (a + b) * normPdf<double>(z, mu, norm_scale);
    double C2 = b / (a + b) * uniform_x;
    const double normalization_constant = C1 + C2;
    C1 /= normalization_constant;
    C2 /= normalization_constant;
    const double f = C1 * (a + 1.0) / (a + b + 1.0) + C2 * a / (a + b + 1.0);
    const double e =
        C1 * (a + 1.0) * (a + 2.0) / ((a + b + 1.0) * (a + b + 2.0)) +
        C2 * a * (a + 1.0) / ((a + b + 1.0) * (a + b + 2.0));

    // update parameters
    const double mu_new = C1 * m + C2 * mu;
    sigma2 = C1 * (s2 + m * m) + C2 * (sigma2 + mu * mu) - mu_new * mu_new;
    mu = mu_new;
    a = (e - f) / (f - e / f);
    b = a * (1.0 - f) / f;

    // TODO: This happens sometimes.
    if (sigma2 < 0.0) {
        std::cout << "Seed sigma2 is negative! \n";
        sigma2 = oldsigma2;
    }
    if (mu < 0.0) {
        std::cout << "Seed diverged! mu is negative!! \n";
        mu = 1.0;
        return false;
    }
    return true;
}

bool updateFilterGaussian(const double z,  // Measurement
                          const double tau2,
                          Eigen::Ref<SeedState>& mu_sigma2_a_b) {
    // double& mu = mu_sigma2_a_b(0);
    // double& sigma2 = mu_sigma2_a_b(1);
    // double& a = mu_sigma2_a_b(2);
    // double& b = mu_sigma2_a_b(3);
    // const double norm_scale = std::sqrt(sigma2 + tau2);
    // if (std::isnan(norm_scale)) {
    //     LOG(WARNING) << "Update Seed: Sigma2+Tau2 is NaN";
    //     return false;
    // }
    // const double denom = (sigma2 + tau2);
    // mu = (sigma2 * z + tau2 * mu) / denom;
    // sigma2 = sigma2 * tau2 / denom;
    // CHECK_GE(sigma2, 0.0);
    // CHECK_GE(mu, 0.0);
    // return true;
}

double computeTau(const Transformation& T_ref_cur,
                  const BearingVector& f,
                  const double z,
                  const double px_error_angle) {
    // const BearingVector& t = T_ref_cur.getPosition();
    // TODO 
    const BearingVector& t = T_ref_cur.translation();
    const BearingVector a = f * z - t;
    double t_norm = t.norm();
    double a_norm = a.norm();
    double alpha = std::acos(f.dot(t) / t_norm);             // dot product
    double beta = std::acos(a.dot(-t) / (t_norm * a_norm));  // dot product
    double beta_plus = beta + px_error_angle;
    double gamma_plus =
        M_PI - alpha - beta_plus;  // triangle angles sum to PI
    double z_plus =
        t_norm * std::sin(beta_plus) / std::sin(gamma_plus);  // law of sines
    return (z_plus - z);                                      // tau
}


bool depthFromTriangulation(const Transformation& T_search_ref,
                                            const Eigen::Vector3d& f_ref,
                                            const Eigen::Vector3d& f_cur,
                                            double* depth) {
    Eigen::Matrix<double, 3, 2> A;
    A << T_search_ref.rotationMatrix() * f_ref, f_cur;
    const Eigen::Matrix2d AtA = A.transpose() * A;
    if (AtA.determinant() < 0.000001) return false;
    const Eigen::Vector2d depth2 =
        -AtA.inverse() * A.transpose() * T_search_ref.translation();
    (*depth) = std::fabs(depth2[0]);
    return true;
}

}  // namespace depth_filter_utils
}  // namespace ORB_SLAM3

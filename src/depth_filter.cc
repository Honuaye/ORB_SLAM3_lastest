#include <algorithm>

#include "depth_filter.h"
#include "ORBmatcher.h"
#include "MapPoint.h"
#include "Atlas.h"

namespace ORB_SLAM3 {
class MapPoint;
class Atlas;

// yhh
// 问题清单：
// 1. 检查初始化种子是否有问题，包括设定的初始值
// 2. 目前选取 10 个 number_to_update_depth_filter_ 用当前帧更新， 但是这 10 个关键帧是上一帧对应 共视关系 的帧
// 3. 选取被更新的关键帧的策略是否有问题
// 3. 更新种子是否有问题 (需要找到一个地方和SVO的projectLocalMap 中 updateseed 一样更多次地更新种子)
    // 3.1 是否存在错误代码
    // 3.2  converged 后为啥重投影误差巨大无比 
    // 3.2 深入理解 ORBSLAM 代码， 确保传入的特征点参数是正确的，例如2D图像点是不是已经去过畸变了的
// 4. 确认了 每张图像 金字塔提取到的特征点都乘以对应缩放存放到 第一层图像上了 (SVO 只 取了 前三层金字塔， 这里是否也要这样)

// 影响因素：
// 1. 图像是否已经去过畸变, SVO 是
// 2. 选取更新的关键帧： 选和当前帧更近的关键帧
// 3. 初始化种子的时候给的初始值很重要，种子 invDepth = 1/depth_median;  sigama2=1/(depth_min*depth_min*36);
// 4. 被 normalize() 和 normalized() 坑了 ！！！ 我擦， normalize 表示自身会归一化；

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
    size_t total_num = 0;
    size_t n_success = 0;
    depth_filter_utils::false_notSeed_ = 0;
    depth_filter_utils::false_in_findEpipolarMatchDirect_ = 0;
    depth_filter_utils::false_in_check_visibility_ = 0;
    depth_filter_utils::false_in_update_ = 0;
    for (auto ref_frame : ref_frames_with_seeds) {
        if(ref_frame->depth_filter_processing_) {
            for(size_t seed_index = 0; seed_index < ref_frame->depth_filter_state_.size(); seed_index++) {
                if(ref_frame->depth_filter_state_.at(seed_index) != seed::SeedType::Seed &&
                    ref_frame->depth_filter_state_.at(seed_index) != seed::SeedType::ConvergedSeed) {
                    continue;
                }
                total_num++;
                if(depth_filter_utils::updateSeedWithFrame(cur_frame, ref_frame, seed_index,
                    options_.seed_convergence_sigma2_thresh, true, false, true)) {
                    n_success++;
                }
            }
        }
    }
    printf("\n");
    std::cout
        <<"total_num = " << total_num
        <<"\t findEpipolarMatchDirect_ = " << depth_filter_utils::false_in_findEpipolarMatchDirect_
        <<", "<<depth_filter_utils::false_in_findEpipolarMatchDirect_/(1.0 * total_num)
        <<"\t check_visibility_ = " << depth_filter_utils::false_in_check_visibility_
        <<", "<<depth_filter_utils::false_in_check_visibility_/(1.0 * total_num)
        <<"\t update_ = " << depth_filter_utils::false_in_update_
        <<", "<<depth_filter_utils::false_in_update_/(1.0 * total_num)
        <<"\t n_success = " << n_success
        <<", "<<n_success/(1.0 * total_num)
        << "\n";
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
    // SVO2:
    // frame_utils::getSceneDepth(new_frames_->at(0), depth_median_, depth_min_,
    //                            depth_max_);
    // depth_filter_->addKeyframe(new_frames_->at(kf_id), depth_median_,
                            //    0.5 * depth_min_, depth_median_ * 1.5);
    // ORBSLAM:
        // double depth_mean = mpCurrentKeyFrame->mPrevKF->ComputeSceneMedianDepth(1);
        // double depth_min = 0.1;
        // double depth_max = depth_mean * 2;
        // depth_filter_->addKeyframe(mpCurrentKeyFrame, depth_mean, depth_min, depth_max);
void initializeSeeds(KeyFramePtr keyframe,
                     const size_t max_n_seeds,
                     const float depth_min,
                     const float depth_max,
                     const float depth_mean) {
    Eigen::Matrix<double, 4, 1> invmu_sigma2_a_b_init;
    invmu_sigma2_a_b_init(0, 0) = seed::getMeanFromDepth(depth_mean);
    invmu_sigma2_a_b_init(1, 0) = seed::getInitSigma2FromMuRange(1.0 / depth_min);
    invmu_sigma2_a_b_init(2, 0) = 10.0;
    invmu_sigma2_a_b_init(3, 0) = 10.0;
    std::cout 
        << "invmu = "<<invmu_sigma2_a_b_init(0, 0)
        << "\t sigma2 = "<<invmu_sigma2_a_b_init(1, 0)
        << "\t depth_min = "<<depth_min
        << "\t depth_max = "<<depth_max
        << "\t depth_mean = "<<depth_mean
        <<"\n";
    // mvpMapPoints ： 不是null时候表示和已有的地图点关联上了； 是null就创建为种子
    // mvpMapPoints 和 mvKeysUn 和 是一样长度并且一一对应的
    const auto & mvpMapPoints = keyframe->GetMapPointMatches();
    for(size_t i = 0; i < mvpMapPoints.size(); ++i) {
        if(mvpMapPoints.at(i)) {
            keyframe->depth_filter_state_.at(i) = seed::SeedType::NotSeed;
            continue;
        }
        Eigen::Matrix<double, 3, 1> f;
        // TODO-yhh : 确认用 mvKeys 还是 mvKeysUn
        // TODO-yhh-check
        // 因为有金字塔，需要确认一下 mvKeys、mvKeysUn 保存的是对应层图像上的像素坐标还是都保存在第一层上
        // SVO 存放的是没有去畸变的原始的2D
        // 如果这里 px 还有畸变， 下面计算 f 部分也要修改
        auto px = keyframe->mvKeysUn.at(i);
        // TODO-yhh-check
        int level = px.octave;
        if(level > 2) { // 只取 0 1 2 三层
            keyframe->depth_filter_state_.at(i) = seed::SeedType::NotSeed;
            continue;
        }
        // std::cout
        //     << "px.pt.x = " << px.pt.x
        //     << ",  px.pt.y = " << px.pt.y
        //     << ",  level = " << level << "; "
        //     ;
        auto x = (px.pt.x - keyframe->cx) / keyframe->fx;
        auto y = (px.pt.y - keyframe->cy) / keyframe->fy;
        f << static_cast<double>(x), static_cast<double>(y), 1;
        f.normalize();
        keyframe->invmu_sigma2_a_b_vec_.block<4, 1>(0, i) = invmu_sigma2_a_b_init;
        keyframe->bearing_vecs_.block<3, 1>(0, i) = f;
        keyframe->depth_filter_state_.at(i) = seed::SeedType::Seed;
        keyframe->seed_mu_range_ = 1.0 / depth_min;
        Keypoint px22;
        px22 << px.pt.x, px.pt.y;
        double score = 0.0;
        FeatureWrapper ft(px22, f, level, score);
        keyframe->feature_wrappers_.at(i) = ft;
    }
    keyframe->depth_filter_processing_ = true;
}

bool updateSeedWithFrame(const FramePtr& cur_frame,
                KeyFramePtr& ref_keyframe,
                const size_t& seed_index,
                const double sigma2_convergence_threshold,
                const bool check_visibility,
                const bool check_convergence,
                const bool use_vogiatzis_update) {
    if(ref_keyframe->depth_filter_state_.at(seed_index) != seed::SeedType::Seed &&
        ref_keyframe->depth_filter_state_.at(seed_index) != seed::SeedType::ConvergedSeed) {
        false_notSeed_++;
        return false;
    }
    // Create wrappers
    // TODO-yhh
    FeatureWrapper ref_ftr = ref_keyframe->feature_wrappers_.at(seed_index);
    Eigen::Ref<SeedState> state = ref_keyframe->invmu_sigma2_a_b_vec_.col(seed_index);
    Transformation T_cur_ref;
    Sophus::SE3d Tcur2_w = cur_frame->GetPose().cast<double>();
    Sophus::SE3d Tref1_w = ref_keyframe->GetPose().cast<double>(); // GetPose : mTcw
    T_cur_ref = Tcur2_w * Tref1_w.inverse();
    // // check if point is visible in the current image
    if (check_visibility) {
        const Eigen::Vector3d xyz_f(T_cur_ref * (seed::getDepth(state) * ref_ftr.f));
        Eigen::Matrix<double, 2, 1> px;
        px = cur_frame->mpCamera->project(xyz_f);
        const int boundary = 9;  // check margin
        if(px(0) < boundary || px(1) < boundary) {
            false_in_check_visibility_++;
            return false;
        }
        if(px(0) > cur_frame->mpORBextractorLeft->mvImagePyramid[0].cols
           ||px(1) > cur_frame->mpORBextractorLeft->mvImagePyramid[0].rows) {
            false_in_check_visibility_++;
           return false;
        }
    }
    // sanity checks
    if (std::isnan(seed::mu(state)) || std::isnan(std::sqrt(seed::sigma2(state)))) {
        // set bad
        ref_keyframe->depth_filter_state_.at(seed_index) = seed::SeedType::NotSeed;
        return false;
    }
    double depth;
    SVOMatcher matcher;
    SVOMatcher::MatchResult res = matcher.findEpipolarMatchDirect(
        *ref_keyframe, *cur_frame, T_cur_ref, ref_ftr, seed::getInvDepth(state),
        seed::getInvMinDepth(state), seed::getInvMaxDepth(state), depth);
    if (res != SVOMatcher::MatchResult::kSuccess) {
        false_in_findEpipolarMatchDirect_++;
        seed::increaseOutlierProbability(state);
        return false;
    }
    // // compute tau
    // // TODO
    constexpr double px_noise = 1.0;
    double px_error_angle;
    // static double px_error_angle = cur_frame.getAngleError(px_noise);
        // return std::atan(img_err/(2.0*fx_)) + std::atan(img_err/(2.0*fy_));
    px_error_angle = std::atan(px_noise/(2.0*cur_frame->fx)) + std::atan(px_noise/(2.0*cur_frame->fy));
    const double depth_sigma = computeTau(T_cur_ref.inverse(),
        ref_keyframe->bearing_vecs_.col(seed_index), depth, px_error_angle);
    // // update the estimate
    if (use_vogiatzis_update) {
        if (!updateFilterVogiatzis(
                seed::getMeanFromDepth(depth),
                seed::getSigma2FromDepthSigma(depth, depth_sigma),
                ref_keyframe->seed_mu_range_, state)) {
            printf("updateFilterVogiatzis bad !!!!!!! \n");
            false_in_update_++;
            ref_keyframe->depth_filter_state_.at(seed_index) = seed::SeedType::NotSeed;
            return false;
        }
    } else {
        if (!updateFilterGaussian(
                seed::getMeanFromDepth(depth),
                seed::getSigma2FromDepthSigma(depth, depth_sigma), state)) {
            printf("updateFilterGaussian bad !!!!!!! \n");
            ref_keyframe->depth_filter_state_.at(seed_index) = seed::SeedType::NotSeed;
            false_in_update_++;
            return false;
        }
    }
    // check if converged
    if (seed::isConverged(state, ref_keyframe->seed_mu_range_, sigma2_convergence_threshold)) {
        // printf("isConverged  ");
        ref_keyframe->depth_filter_state_.at(seed_index) = seed::SeedType::ConvergedSeed;
    }
    ref_keyframe->invmu_sigma2_a_b_vec_.block<4, 1>(0, seed_index) = state;
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
    size_t already_converge = 0;

    for(const auto& match : vMatchedPairs) {
        auto cur_index = match.first;
        auto ref_index = match.second;
        // if(!ref_keyframe->depth_filter_state_.at(ref_index)) continue;
        if(ref_keyframe->depth_filter_state_.at(ref_index) == seed::SeedType::NotSeed) {
            continue;
        } else if(ref_keyframe->depth_filter_state_.at(ref_index) == seed::SeedType::ConvergedSeed) {
            converged_num ++;
            already_converge++;
            // Eigen::Ref<SeedState> state = ref_keyframe->invmu_sigma2_a_b_vec_.col(ref_index);
            Eigen::Vector3d p_ref = ref_keyframe->bearing_vecs_.col(ref_index) * ref_keyframe->getSeedDepth(ref_index);
            auto T_wc = ref_keyframe->GetPoseInverse();
            Eigen::Vector3f x3D = T_wc.rotationMatrix() * p_ref.cast<float>() + T_wc.translation();
            double project_e1 = 0.0;
            double project_e2 = 0.0;
                Transformation T_cur_ref;
                Sophus::SE3d Tcur2_w = cur_keyframe->GetPose().cast<double>();
                Sophus::SE3d Tref1_w = ref_keyframe->GetPose().cast<double>(); // mTcw
                T_cur_ref = Tcur2_w * Tref1_w.inverse();
            Eigen::Vector3d p_cur = T_cur_ref.rotationMatrix() * p_ref + T_cur_ref.translation();
            {
                auto kp1 = ref_keyframe->mvKeysUn[ref_index];
                auto uv11 = ref_keyframe->mpCamera->project(p_ref);
                auto uv1 = uv11.cast<float>();
                float errX1 = uv1(0) - kp1.pt.x;
                float errY1 = uv1(1) - kp1.pt.y;
                project_e1 = errX1 * errX1 + errY1 * errY1;

                auto kp2 = cur_keyframe->mvKeysUn[cur_index];
                auto uv22 = cur_keyframe->mpCamera->project(p_cur);
                auto uv2 = uv22.cast<float>();
                float errX2 = uv2(0) - kp2.pt.x;
                float errY2 = uv2(1) - kp2.pt.y;
                project_e2 = errX2 * errX2 + errY2 * errY2;
            }
            MapPoint* pMP = new MapPoint(x3D, cur_keyframe, mpAtlas->GetCurrentMap());
            std::cout
                << "pMP. = " << pMP->GetWorldPos().transpose()
                << "\t project_e1 = " << project_e1
                << "\t project_e2 = " << project_e2
                <<"\t sigma2 = " << ref_keyframe->invmu_sigma2_a_b_vec_.col(ref_index)(1)
                << "\n";
            pMP->AddObservation(cur_keyframe, cur_index);
            pMP->AddObservation(ref_keyframe, ref_index);
            cur_keyframe->AddMapPoint(pMP, cur_index);
            ref_keyframe->AddMapPoint(pMP, ref_index);
            pMP->ComputeDistinctiveDescriptors();
            pMP->UpdateNormalAndDepth();
            mpAtlas->AddMapPoint(pMP);
            cur_keyframe->depth_filter_state_.at(cur_index) = seed::SeedType::alreadyChangeToPoint;
            ref_keyframe->depth_filter_state_.at(ref_index) = seed::SeedType::alreadyChangeToPoint;
            ref_keyframe->printSeedType();
            continue;
        }
    }
    // std::cout 
    //     << "vMatchedPairs.size  = " << vMatchedPairs.size()
    //     << "\t converged_num = " << converged_num
    //     // << "\t already_converge = " << already_converge
    //     ;
    //     // << "\n";
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

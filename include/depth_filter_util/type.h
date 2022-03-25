#pragma once

#include <memory>
#include <Eigen/Core>
#include <sophus/se3.hpp>

#include "GeometricCamera.h"
#include "Frame.h"
#include "KeyFrame.h"
// #include "depth_filter_util/seed.h"
// #include "depth_filter_util/SVOmatcher.h"
// #include "SVOmatcher.h"
// #include "depth_filter.h"

namespace ORB_SLAM3 {

class Frame;
class KeyFrame;
class GeometricCamera;
using CameraPtr = GeometricCamera*;
using GradientVector = Eigen::Matrix<double, 2, 1>;
using Transformation = Sophus::SE3<double>;
using Keypoint = Eigen::Matrix<double, 2, 1>;

// for seed
using SeedState = Eigen::Matrix<double, 4, 1>;
using SeedStateVec = Eigen::Matrix<double, 4, Eigen::Dynamic, Eigen::ColMajor>;

// TODO
// class Point;
// using PointPtr = std::shared_ptr<Point>;


// using FramePtr = std::shared_ptr<Frame>;
using FramePtr = Frame*;
// using KeyFramePtr = std::shared_ptr<KeyFrame>;
using KeyFramePtr = KeyFrame*;
using Transformation = Sophus::SE3<double>;
using BearingVector = Eigen::Matrix<double, 3, 1>;
using GradientVector = Eigen::Matrix<double, 2, 1>;


}  // namespace ORB_SLAM3

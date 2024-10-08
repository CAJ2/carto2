/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer/mapping/internal/3d/scan_matching/ceres_scan_matcher_3d.h"

#include <memory>

#include "Eigen/Core"
#include "cartographer/common/internal/testing/lua_parameter_dictionary_test_helpers.h"
#include "cartographer/mapping/3d/hybrid_grid.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/rigid_transform_test_helpers.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {
namespace {

class CeresScanMatcher3DTest : public ::testing::Test {
 protected:
  CeresScanMatcher3DTest()
      : hybrid_grid_(1.f),
        intensity_hybrid_grid_(1.f),
        expected_pose_(
            transform::Rigid3d::Translation(Eigen::Vector3d(-1., 0., 0.))) {
    std::vector<sensor::RangefinderPoint> points;
    std::vector<float> intensities;
    for (const Eigen::Vector3f& point :
         {Eigen::Vector3f(-3.f, 2.f, 0.f), Eigen::Vector3f(-4.f, 2.f, 0.f),
          Eigen::Vector3f(-5.f, 2.f, 0.f), Eigen::Vector3f(-6.f, 2.f, 0.f),
          Eigen::Vector3f(-6.f, 3.f, 1.f), Eigen::Vector3f(-6.f, 4.f, 2.f),
          Eigen::Vector3f(-7.f, 3.f, 1.f)}) {
      points.push_back({point});
      intensities.push_back(50);
      hybrid_grid_.SetProbability(
          hybrid_grid_.GetCellIndex(expected_pose_.cast<float>() * point), 1.);
      intensity_hybrid_grid_.AddIntensity(
          intensity_hybrid_grid_.GetCellIndex(expected_pose_.cast<float>() *
                                              point),
          50);
    }
    point_cloud_ = sensor::PointCloud(points, intensities);

    auto parameter_dictionary = common::MakeDictionary(R"text(
        return {
          occupied_space_weight_0 = 1.,
          intensity_cost_function_options_0 = {
            weight = 0.5,
            huber_scale = 55,
            intensity_threshold = 100,
          },
          translation_weight = 0.01,
          rotation_weight = 0.1,
          only_optimize_yaw = false,
          ceres_solver_options = {
            use_nonmonotonic_steps = true,
            max_num_iterations = 10,
            num_threads = 1,
          },
        })text");
    options_ = CreateCeresScanMatcherOptions3D(parameter_dictionary.get());
    ceres_scan_matcher_.reset(new CeresScanMatcher3D(options_));
  }

  void TestFromInitialPose(const transform::Rigid3d& initial_pose) {
    transform::Rigid3d pose;

    ceres::Solver::Summary summary;

    IntensityHybridGrid* intensity_hybrid_grid_ptr =
        point_cloud_.intensities().empty() ? nullptr : &intensity_hybrid_grid_;

    ceres_scan_matcher_->Match(
        initial_pose.translation(), initial_pose,
        {{&point_cloud_, &hybrid_grid_, intensity_hybrid_grid_ptr}}, &pose,
        &summary);
    EXPECT_NEAR(0., summary.final_cost, 1e-2) << summary.FullReport();
    EXPECT_THAT(pose, transform::IsNearly(expected_pose_, 3e-2));
  }

  HybridGrid hybrid_grid_;
  IntensityHybridGrid intensity_hybrid_grid_;
  transform::Rigid3d expected_pose_;
  sensor::PointCloud point_cloud_;
  cartographer_proto::mapping::scan_matching::CeresScanMatcherOptions3D
      options_;
  std::unique_ptr<CeresScanMatcher3D> ceres_scan_matcher_;
};

TEST_F(CeresScanMatcher3DTest, PerfectEstimate) {
  TestFromInitialPose(
      transform::Rigid3d::Translation(Eigen::Vector3d(-1., 0., 0.)));
}

TEST_F(CeresScanMatcher3DTest, AlongX) {
  ceres_scan_matcher_.reset(new CeresScanMatcher3D(options_));
  TestFromInitialPose(
      transform::Rigid3d::Translation(Eigen::Vector3d(-0.8, 0., 0.)));
}

TEST_F(CeresScanMatcher3DTest, AlongZ) {
  TestFromInitialPose(
      transform::Rigid3d::Translation(Eigen::Vector3d(-1., 0., -0.2)));
}

TEST_F(CeresScanMatcher3DTest, AlongXYZ) {
  TestFromInitialPose(
      transform::Rigid3d::Translation(Eigen::Vector3d(-0.9, -0.2, 0.2)));
}

TEST_F(CeresScanMatcher3DTest, FullPoseCorrection) {
  // We try to find the rotation around z...
  const auto additional_transform = transform::Rigid3d::Rotation(
      Eigen::AngleAxisd(0.05, Eigen::Vector3d(0., 0., 1.)));
  point_cloud_ = sensor::TransformPointCloud(
      point_cloud_, additional_transform.cast<float>());
  expected_pose_ = expected_pose_ * additional_transform.inverse();
  // ...starting initially with rotation around x.
  TestFromInitialPose(
      transform::Rigid3d(Eigen::Vector3d(-0.95, -0.05, 0.05),
                         Eigen::AngleAxisd(0.05, Eigen::Vector3d(1., 0., 0.))));
}

}  // namespace
}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer

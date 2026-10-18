#include <cmath>

#include <gtest/gtest.h>

#include "table_plane_detector/plane_estimator.hpp"

using table_plane_detector::PlaneEstimator;

TEST(PlaneMath, NormalizesAndUsesDeterministicSign)
{
  const Eigen::Vector4d plane = PlaneEstimator::normalize_plane(
    Eigen::Vector4d(0.0, 0.0, 2.0, -4.0));
  EXPECT_NEAR(plane.head<3>().norm(), 1.0, 1.0e-12);
  EXPECT_GE(plane[3], 0.0);
  EXPECT_NEAR(plane[2], -1.0, 1.0e-12);
  EXPECT_NEAR(plane[3], 2.0, 1.0e-12);
}

TEST(PlaneMath, ComputesDistances)
{
  const Eigen::Vector4d plane(0.0, -0.5, -0.866025403784, 1.0);
  EXPECT_NEAR(PlaneEstimator::point_plane_distance(
      plane, Eigen::Vector3d(0.0, 0.0, 1.0 / 0.866025403784)), 0.0, 1.0e-9);
  EXPECT_NEAR(PlaneEstimator::optical_axis_distance(plane), 1.15470053838, 1.0e-9);
}

TEST(PlaneMath, RejectsParallelOpticalAxis)
{
  const Eigen::Vector4d plane(1.0, 0.0, 0.0, 1.0);
  EXPECT_TRUE(std::isnan(PlaneEstimator::optical_axis_distance(plane)));
}

TEST(PlaneMath, NormalAngleIgnoresSign)
{
  EXPECT_NEAR(PlaneEstimator::normal_angle_deg(
      Eigen::Vector3d::UnitZ(), -Eigen::Vector3d::UnitZ()), 0.0, 1.0e-12);
  EXPECT_NEAR(PlaneEstimator::normal_angle_deg(
      Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitX()), 90.0, 1.0e-12);
}

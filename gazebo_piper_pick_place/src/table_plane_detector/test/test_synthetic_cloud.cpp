#include <cmath>
#include <random>

#include <gtest/gtest.h>

#include "table_plane_detector/plane_estimator.hpp"

using table_plane_detector::EstimatorParameters;
using table_plane_detector::PlaneEstimator;

TEST(SyntheticCloud, FindsTiltedTableAmongOutliers)
{
  EstimatorParameters parameters;
  parameters.depth_min_m = 0.2;
  parameters.depth_max_m = 2.0;
  parameters.crop_min_x_m = -1.0;
  parameters.crop_max_x_m = 1.0;
  parameters.crop_min_y_m = -1.0;
  parameters.crop_max_y_m = 1.0;
  parameters.voxel_size_m = 0.003;
  parameters.ransac_distance_threshold_m = 0.006;
  parameters.minimum_inlier_count = 1000;
  parameters.minimum_inlier_ratio = 0.40;
  parameters.maximum_normal_error_deg = 8.0;
  parameters.minimum_plane_distance_m = 0.8;
  parameters.maximum_plane_distance_m = 1.2;
  parameters.maximum_rms_error_m = 0.005;
  parameters.minimum_plane_extent_m = 0.4;

  auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  std::mt19937 generator(42);
  std::normal_distribution<double> noise(0.0, 0.0015);
  for (int yi = -35; yi <= 35; ++yi) {
    for (int xi = -45; xi <= 45; ++xi) {
      const double x = xi * 0.012;
      const double y = yi * 0.012;
      const double z = (1.0 - 0.5 * y) / 0.866025403784 + noise(generator);
      cloud->push_back(pcl::PointXYZ(x, y, z));
    }
  }
  std::uniform_real_distribution<float> uniform(-0.75F, 0.75F);
  for (int index = 0; index < 1800; ++index) {
    cloud->push_back(pcl::PointXYZ(
        uniform(generator), uniform(generator), 0.3F + std::abs(uniform(generator))));
  }

  const Eigen::Vector3d expected(0.0, -0.5, -0.866025403784);
  const auto result = PlaneEstimator(parameters).estimate(cloud, expected);
  ASSERT_TRUE(result.valid) << result.status;
  EXPECT_NEAR(result.perpendicular_distance_m, 1.0, 0.006);
  EXPECT_NEAR(result.optical_axis_distance_m, 1.15470053838, 0.008);
  EXPECT_LT(result.normal_error_deg, 1.0);
  EXPECT_LT(result.rms_error_m, 0.004);
}

TEST(SyntheticCloud, RejectsSmallPlane)
{
  EstimatorParameters parameters;
  parameters.depth_min_m = 0.2;
  parameters.depth_max_m = 2.0;
  parameters.voxel_size_m = 0.0;
  parameters.minimum_inlier_count = 10;
  parameters.minimum_inlier_ratio = 0.5;
  parameters.minimum_plane_distance_m = 0.5;
  parameters.maximum_plane_distance_m = 1.5;
  parameters.minimum_plane_extent_m = 0.5;

  auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  for (int y = -5; y <= 5; ++y) {
    for (int x = -5; x <= 5; ++x) {
      cloud->push_back(pcl::PointXYZ(x * 0.01F, y * 0.01F, 1.0F));
    }
  }
  const auto result = PlaneEstimator(parameters).estimate(
    cloud, Eigen::Vector3d(0.0, 0.0, -1.0));
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.status, "PLANE_TOO_SMALL");
}

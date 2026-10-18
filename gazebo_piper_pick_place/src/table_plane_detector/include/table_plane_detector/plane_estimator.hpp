#pragma once

#include <cstddef>
#include <limits>
#include <string>

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace table_plane_detector
{

struct EstimatorParameters
{
  double depth_min_m{0.30};
  double depth_max_m{2.00};
  double crop_min_x_m{-0.80};
  double crop_max_x_m{0.80};
  double crop_min_y_m{-0.80};
  double crop_max_y_m{0.80};
  double voxel_size_m{0.005};
  double ransac_distance_threshold_m{0.008};
  int max_iterations{500};
  std::size_t minimum_inlier_count{5000};
  double minimum_inlier_ratio{0.25};
  double maximum_normal_error_deg{10.0};
  double minimum_plane_distance_m{0.40};
  double maximum_plane_distance_m{1.50};
  double maximum_rms_error_m{0.008};
  double minimum_plane_extent_m{0.30};
};

struct PlaneEstimate
{
  Eigen::Vector4d coefficients{Eigen::Vector4d::Zero()};
  double perpendicular_distance_m{std::numeric_limits<double>::quiet_NaN()};
  double optical_axis_distance_m{std::numeric_limits<double>::quiet_NaN()};
  std::size_t input_point_count{0};
  std::size_t filtered_point_count{0};
  std::size_t inlier_count{0};
  double inlier_ratio{0.0};
  double rms_error_m{0.0};
  double normal_error_deg{0.0};
  double extent_u_m{0.0};
  double extent_v_m{0.0};
  bool valid{false};
  std::string status{"UNINITIALIZED"};
  pcl::PointCloud<pcl::PointXYZ>::Ptr inliers{
    new pcl::PointCloud<pcl::PointXYZ>()};
};

class PlaneEstimator
{
public:
  explicit PlaneEstimator(EstimatorParameters parameters = {});

  PlaneEstimate estimate(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud,
    const Eigen::Vector3d & expected_normal) const;

  const EstimatorParameters & parameters() const {return parameters_;}

  static Eigen::Vector4d normalize_plane(const Eigen::Vector4d & plane);
  static double point_plane_distance(
    const Eigen::Vector4d & normalized_plane,
    const Eigen::Vector3d & point);
  static double optical_axis_distance(const Eigen::Vector4d & normalized_plane);
  static double normal_angle_deg(
    const Eigen::Vector3d & first,
    const Eigen::Vector3d & second);

private:
  static Eigen::Vector4d refine_plane(
    const pcl::PointCloud<pcl::PointXYZ> & inliers);
  static void calculate_extent(
    const pcl::PointCloud<pcl::PointXYZ> & inliers,
    const Eigen::Vector3d & normal,
    double & extent_u_m,
    double & extent_v_m);

  EstimatorParameters parameters_;
};

}  // namespace table_plane_detector

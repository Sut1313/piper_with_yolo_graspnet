#include "table_plane_detector/plane_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include <Eigen/Eigenvalues>
#include <pcl/common/centroid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>

namespace table_plane_detector
{
namespace
{
constexpr double kRadiansToDegrees = 180.0 / M_PI;
constexpr double kAxisEpsilon = 1.0e-9;
}

PlaneEstimator::PlaneEstimator(EstimatorParameters parameters)
: parameters_(std::move(parameters))
{
}

Eigen::Vector4d PlaneEstimator::normalize_plane(const Eigen::Vector4d & plane)
{
  Eigen::Vector4d result = plane;
  const double norm = result.head<3>().norm();
  if (!std::isfinite(norm) || norm < kAxisEpsilon) {
    return Eigen::Vector4d::Zero();
  }
  result /= norm;
  if (result[3] < 0.0) {
    result = -result;
  }
  return result;
}

double PlaneEstimator::point_plane_distance(
  const Eigen::Vector4d & normalized_plane,
  const Eigen::Vector3d & point)
{
  return std::abs(normalized_plane.head<3>().dot(point) + normalized_plane[3]);
}

double PlaneEstimator::optical_axis_distance(const Eigen::Vector4d & normalized_plane)
{
  const double c = normalized_plane[2];
  if (std::abs(c) < kAxisEpsilon) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const double distance = -normalized_plane[3] / c;
  if (!std::isfinite(distance) || distance <= 0.0) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return distance;
}

double PlaneEstimator::normal_angle_deg(
  const Eigen::Vector3d & first,
  const Eigen::Vector3d & second)
{
  if (first.norm() < kAxisEpsilon || second.norm() < kAxisEpsilon) {
    return std::numeric_limits<double>::infinity();
  }
  const double dot = std::clamp(
    std::abs(first.normalized().dot(second.normalized())), 0.0, 1.0);
  return std::acos(dot) * kRadiansToDegrees;
}

Eigen::Vector4d PlaneEstimator::refine_plane(
  const pcl::PointCloud<pcl::PointXYZ> & inliers)
{
  if (inliers.size() < 3) {
    return Eigen::Vector4d::Zero();
  }

  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  for (const auto & point : inliers) {
    centroid += Eigen::Vector3d(point.x, point.y, point.z);
  }
  centroid /= static_cast<double>(inliers.size());

  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (const auto & point : inliers) {
    const Eigen::Vector3d centered =
      Eigen::Vector3d(point.x, point.y, point.z) - centroid;
    covariance.noalias() += centered * centered.transpose();
  }
  covariance /= static_cast<double>(inliers.size());

  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
  if (solver.info() != Eigen::Success) {
    return Eigen::Vector4d::Zero();
  }
  const Eigen::Vector3d normal = solver.eigenvectors().col(0).normalized();
  Eigen::Vector4d plane;
  plane << normal, -normal.dot(centroid);
  return normalize_plane(plane);
}

void PlaneEstimator::calculate_extent(
  const pcl::PointCloud<pcl::PointXYZ> & inliers,
  const Eigen::Vector3d & normal,
  double & extent_u_m,
  double & extent_v_m)
{
  extent_u_m = 0.0;
  extent_v_m = 0.0;
  if (inliers.empty() || normal.norm() < kAxisEpsilon) {
    return;
  }

  const Eigen::Vector3d u = normal.normalized().unitOrthogonal();
  const Eigen::Vector3d v = normal.normalized().cross(u).normalized();
  double min_u = std::numeric_limits<double>::infinity();
  double max_u = -std::numeric_limits<double>::infinity();
  double min_v = std::numeric_limits<double>::infinity();
  double max_v = -std::numeric_limits<double>::infinity();
  for (const auto & point : inliers) {
    const Eigen::Vector3d p(point.x, point.y, point.z);
    const double pu = p.dot(u);
    const double pv = p.dot(v);
    min_u = std::min(min_u, pu);
    max_u = std::max(max_u, pu);
    min_v = std::min(min_v, pv);
    max_v = std::max(max_v, pv);
  }
  extent_u_m = max_u - min_u;
  extent_v_m = max_v - min_v;
}

PlaneEstimate PlaneEstimator::estimate(
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud,
  const Eigen::Vector3d & expected_normal) const
{
  PlaneEstimate result;
  if (!cloud) {
    result.status = "NO_CLOUD";
    return result;
  }
  result.input_point_count = cloud->size();
  if (cloud->size() < 3 || expected_normal.norm() < kAxisEpsilon) {
    result.status = "INSUFFICIENT_POINTS";
    return result;
  }

  auto cropped = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  cropped->reserve(cloud->size());
  for (const auto & point : *cloud) {
    if (!pcl::isFinite(point)) {
      continue;
    }
    if (point.z < parameters_.depth_min_m || point.z > parameters_.depth_max_m ||
      point.x < parameters_.crop_min_x_m || point.x > parameters_.crop_max_x_m ||
      point.y < parameters_.crop_min_y_m || point.y > parameters_.crop_max_y_m)
    {
      continue;
    }
    cropped->push_back(point);
  }

  auto filtered = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  if (parameters_.voxel_size_m > 0.0) {
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    const float leaf = static_cast<float>(parameters_.voxel_size_m);
    voxel.setInputCloud(cropped);
    voxel.setLeafSize(leaf, leaf, leaf);
    voxel.filter(*filtered);
  } else {
    *filtered = *cropped;
  }
  result.filtered_point_count = filtered->size();
  if (filtered->size() < 3) {
    result.status = "INSUFFICIENT_POINTS";
    return result;
  }

  pcl::SACSegmentation<pcl::PointXYZ> segmentation;
  segmentation.setOptimizeCoefficients(true);
  segmentation.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  segmentation.setMethodType(pcl::SAC_RANSAC);
  segmentation.setAxis(expected_normal.normalized().cast<float>());
  segmentation.setEpsAngle(
    parameters_.maximum_normal_error_deg / kRadiansToDegrees);
  segmentation.setDistanceThreshold(parameters_.ransac_distance_threshold_m);
  segmentation.setMaxIterations(parameters_.max_iterations);
  segmentation.setInputCloud(filtered);

  pcl::PointIndices::Ptr indices(new pcl::PointIndices());
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
  segmentation.segment(*indices, *coefficients);
  if (indices->indices.size() < 3 || coefficients->values.size() < 4) {
    result.status = "NO_PLANE";
    return result;
  }

  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(filtered);
  extract.setIndices(indices);
  extract.setNegative(false);
  extract.filter(*result.inliers);

  result.inlier_count = result.inliers->size();
  result.inlier_ratio = static_cast<double>(result.inlier_count) /
    static_cast<double>(result.filtered_point_count);
  result.coefficients = refine_plane(*result.inliers);
  if (result.coefficients.head<3>().norm() < kAxisEpsilon) {
    result.status = "REFINEMENT_FAILED";
    return result;
  }

  result.perpendicular_distance_m = result.coefficients[3];
  result.optical_axis_distance_m = optical_axis_distance(result.coefficients);
  result.normal_error_deg = normal_angle_deg(
    result.coefficients.head<3>(), expected_normal);
  calculate_extent(
    *result.inliers, result.coefficients.head<3>(), result.extent_u_m, result.extent_v_m);

  double squared_error_sum = 0.0;
  for (const auto & point : *result.inliers) {
    const double distance = point_plane_distance(
      result.coefficients, Eigen::Vector3d(point.x, point.y, point.z));
    squared_error_sum += distance * distance;
  }
  result.rms_error_m = std::sqrt(
    squared_error_sum / static_cast<double>(result.inlier_count));

  if (result.inlier_count < parameters_.minimum_inlier_count) {
    result.status = "INSUFFICIENT_INLIERS";
  } else if (result.inlier_ratio < parameters_.minimum_inlier_ratio) {
    result.status = "LOW_INLIER_RATIO";
  } else if (result.normal_error_deg > parameters_.maximum_normal_error_deg) {
    result.status = "NORMAL_REJECTED";
  } else if (
    result.perpendicular_distance_m < parameters_.minimum_plane_distance_m ||
    result.perpendicular_distance_m > parameters_.maximum_plane_distance_m)
  {
    result.status = "DISTANCE_REJECTED";
  } else if (result.rms_error_m > parameters_.maximum_rms_error_m) {
    result.status = "HIGH_RMS_ERROR";
  } else if (
    std::min(result.extent_u_m, result.extent_v_m) < parameters_.minimum_plane_extent_m)
  {
    result.status = "PLANE_TOO_SMALL";
  } else {
    result.valid = true;
    result.status = "VALID";
  }
  return result;
}

}  // namespace table_plane_detector

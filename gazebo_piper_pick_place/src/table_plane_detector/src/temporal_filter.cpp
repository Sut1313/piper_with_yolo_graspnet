#include "table_plane_detector/temporal_filter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace table_plane_detector
{

TemporalFilter::TemporalFilter(TemporalParameters parameters)
: parameters_(std::move(parameters))
{
  parameters_.window_size = std::max<std::size_t>(1, parameters_.window_size);
  parameters_.minimum_tracking_frames = std::max<std::size_t>(
    1, parameters_.minimum_tracking_frames);
  parameters_.maximum_failed_frames = std::max<std::size_t>(
    1, parameters_.maximum_failed_frames);
}

void TemporalFilter::reset()
{
  history_.clear();
  consecutive_valid_ = 0;
  consecutive_failed_ = 0;
}

double TemporalFilter::median(std::vector<double> values)
{
  if (values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const auto middle = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
  std::nth_element(values.begin(), middle, values.end());
  const double upper = *middle;
  if (values.size() % 2 != 0) {
    return upper;
  }
  const auto lower_it = std::max_element(values.begin(), middle);
  return (*lower_it + upper) * 0.5;
}

PlaneEstimate TemporalFilter::filtered_estimate(const PlaneEstimate & newest) const
{
  PlaneEstimate output = newest;
  Eigen::Vector3d normal_sum = Eigen::Vector3d::Zero();
  std::vector<double> distances;
  distances.reserve(history_.size());
  const Eigen::Vector3d reference = history_.front().coefficients.head<3>();
  for (const auto & estimate : history_) {
    Eigen::Vector3d normal = estimate.coefficients.head<3>();
    if (normal.dot(reference) < 0.0) {
      normal = -normal;
    }
    normal_sum += normal;
    distances.push_back(estimate.perpendicular_distance_m);
  }
  if (normal_sum.norm() > 1.0e-9) {
    normal_sum.normalize();
  } else {
    normal_sum = newest.coefficients.head<3>();
  }
  double distance = median(std::move(distances));
  output.coefficients << normal_sum, distance;
  output.coefficients = PlaneEstimator::normalize_plane(output.coefficients);
  output.perpendicular_distance_m = output.coefficients[3];
  output.optical_axis_distance_m = PlaneEstimator::optical_axis_distance(
    output.coefficients);
  output.valid = true;
  output.status = "TRACKING";
  return output;
}

PlaneEstimate TemporalFilter::update(const PlaneEstimate & raw)
{
  PlaneEstimate output = raw;
  if (!raw.valid) {
    consecutive_valid_ = 0;
    ++consecutive_failed_;
    output.valid = false;
    if (consecutive_failed_ >= parameters_.maximum_failed_frames) {
      history_.clear();
      output.status = "LOST:" + raw.status;
    }
    return output;
  }

  if (!history_.empty()) {
    const PlaneEstimate & previous = history_.back();
    const double distance_jump = std::abs(
      raw.perpendicular_distance_m - previous.perpendicular_distance_m);
    const double normal_jump = PlaneEstimator::normal_angle_deg(
      raw.coefficients.head<3>(), previous.coefficients.head<3>());
    if (distance_jump > parameters_.maximum_distance_jump_m ||
      normal_jump > parameters_.maximum_normal_jump_deg)
    {
      consecutive_valid_ = 0;
      ++consecutive_failed_;
      output.valid = false;
      output.status = "TEMPORAL_JUMP";
      if (consecutive_failed_ >= parameters_.maximum_failed_frames) {
        history_.clear();
        output.status = "LOST:TEMPORAL_JUMP";
      }
      return output;
    }
  }

  consecutive_failed_ = 0;
  ++consecutive_valid_;
  history_.push_back(raw);
  while (history_.size() > parameters_.window_size) {
    history_.pop_front();
  }
  if (consecutive_valid_ < parameters_.minimum_tracking_frames) {
    output.valid = false;
    output.status = "INITIALIZING";
    return output;
  }
  return filtered_estimate(raw);
}

}  // namespace table_plane_detector

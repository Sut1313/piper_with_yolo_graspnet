#pragma once

#include <cstddef>
#include <deque>
#include <vector>

#include "table_plane_detector/plane_estimator.hpp"

namespace table_plane_detector
{

struct TemporalParameters
{
  std::size_t window_size{20};
  std::size_t minimum_tracking_frames{5};
  std::size_t maximum_failed_frames{5};
  double maximum_distance_jump_m{0.03};
  double maximum_normal_jump_deg{5.0};
};

class TemporalFilter
{
public:
  explicit TemporalFilter(TemporalParameters parameters = {});

  PlaneEstimate update(const PlaneEstimate & raw);
  void reset();

private:
  PlaneEstimate filtered_estimate(const PlaneEstimate & newest) const;
  static double median(std::vector<double> values);

  TemporalParameters parameters_;
  std::deque<PlaneEstimate> history_;
  std::size_t consecutive_valid_{0};
  std::size_t consecutive_failed_{0};
};

}  // namespace table_plane_detector

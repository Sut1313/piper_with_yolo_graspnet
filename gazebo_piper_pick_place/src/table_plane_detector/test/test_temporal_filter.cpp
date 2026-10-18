#include <cmath>

#include <gtest/gtest.h>

#include "table_plane_detector/temporal_filter.hpp"

using table_plane_detector::PlaneEstimate;
using table_plane_detector::TemporalFilter;
using table_plane_detector::TemporalParameters;

namespace
{
PlaneEstimate estimate(double distance)
{
  PlaneEstimate value;
  value.coefficients << 0.0, 0.0, -1.0, distance;
  value.perpendicular_distance_m = distance;
  value.optical_axis_distance_m = distance;
  value.valid = true;
  value.status = "VALID";
  return value;
}
}  // namespace

TEST(TemporalFilter, RequiresStableStartup)
{
  TemporalParameters parameters;
  parameters.minimum_tracking_frames = 3;
  parameters.window_size = 5;
  TemporalFilter filter(parameters);

  EXPECT_FALSE(filter.update(estimate(1.000)).valid);
  EXPECT_FALSE(filter.update(estimate(1.002)).valid);
  const auto result = filter.update(estimate(0.999));
  EXPECT_TRUE(result.valid);
  EXPECT_EQ(result.status, "TRACKING");
  EXPECT_NEAR(result.perpendicular_distance_m, 1.0, 0.002);
}

TEST(TemporalFilter, RejectsJumpAndEventuallyLosesTrack)
{
  TemporalParameters parameters;
  parameters.minimum_tracking_frames = 1;
  parameters.maximum_failed_frames = 2;
  parameters.maximum_distance_jump_m = 0.02;
  TemporalFilter filter(parameters);

  ASSERT_TRUE(filter.update(estimate(1.0)).valid);
  auto result = filter.update(estimate(1.1));
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.status, "TEMPORAL_JUMP");
  result = filter.update(estimate(1.1));
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.status, "LOST:TEMPORAL_JUMP");
}

TEST(TemporalFilter, InvalidMeasurementIsNeverPublishedAsCurrentValidData)
{
  TemporalParameters parameters;
  parameters.minimum_tracking_frames = 1;
  TemporalFilter filter(parameters);
  ASSERT_TRUE(filter.update(estimate(1.0)).valid);

  PlaneEstimate invalid;
  invalid.status = "NO_PLANE";
  const auto result = filter.update(invalid);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.status, "NO_PLANE");
  EXPECT_TRUE(std::isnan(result.perpendicular_distance_m));
  EXPECT_TRUE(std::isnan(result.optical_axis_distance_m));
}

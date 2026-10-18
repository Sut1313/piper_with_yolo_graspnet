#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <sstream>
#include <string>

#include <Eigen/Geometry>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>

#include "table_plane_detector/msg/table_plane.hpp"
#include "table_plane_detector/plane_estimator.hpp"
#include "table_plane_detector/temporal_filter.hpp"

namespace table_plane_detector
{
namespace
{

template<typename ValueT>
diagnostic_msgs::msg::KeyValue key_value(const std::string & key, const ValueT & value)
{
  diagnostic_msgs::msg::KeyValue result;
  result.key = key;
  std::ostringstream stream;
  stream << value;
  result.value = stream.str();
  return result;
}

}  // namespace

class TablePlaneDetectorNode : public rclcpp::Node
{
public:
  TablePlaneDetectorNode()
  : Node("table_plane_detector"),
    tf_buffer_(std::make_unique<tf2_ros::Buffer>(get_clock())),
    tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
  {
    const double processing_rate_hz = declare_parameter("processing_rate_hz", 5.0);
    minimum_processing_period_ = std::chrono::duration<double>(
      1.0 / std::max(0.1, processing_rate_hz));

    EstimatorParameters estimator_parameters;
    estimator_parameters.depth_min_m = declare_parameter("depth_min_m", 0.30);
    estimator_parameters.depth_max_m = declare_parameter("depth_max_m", 2.00);
    estimator_parameters.crop_min_x_m = declare_parameter("crop_min_x_m", -0.80);
    estimator_parameters.crop_max_x_m = declare_parameter("crop_max_x_m", 0.80);
    estimator_parameters.crop_min_y_m = declare_parameter("crop_min_y_m", -0.80);
    estimator_parameters.crop_max_y_m = declare_parameter("crop_max_y_m", 0.80);
    estimator_parameters.voxel_size_m = declare_parameter("voxel_size_m", 0.005);
    estimator_parameters.ransac_distance_threshold_m = declare_parameter(
      "ransac_distance_threshold_m", 0.008);
    estimator_parameters.max_iterations = declare_parameter("max_iterations", 500);
    estimator_parameters.minimum_inlier_count = static_cast<std::size_t>(
      std::max<std::int64_t>(0, declare_parameter<std::int64_t>("minimum_inlier_count", 5000)));
    estimator_parameters.minimum_inlier_ratio = declare_parameter(
      "minimum_inlier_ratio", 0.25);
    estimator_parameters.maximum_normal_error_deg = declare_parameter(
      "maximum_normal_error_deg", 10.0);
    estimator_parameters.minimum_plane_distance_m = declare_parameter(
      "minimum_plane_distance_m", 0.40);
    estimator_parameters.maximum_plane_distance_m = declare_parameter(
      "maximum_plane_distance_m", 1.50);
    estimator_parameters.maximum_rms_error_m = declare_parameter(
      "maximum_rms_error_m", 0.008);
    estimator_parameters.minimum_plane_extent_m = declare_parameter(
      "minimum_plane_extent_m", 0.30);
    estimator_ = std::make_unique<PlaneEstimator>(estimator_parameters);

    TemporalParameters temporal_parameters;
    temporal_parameters.window_size = static_cast<std::size_t>(
      std::max<std::int64_t>(1, declare_parameter<std::int64_t>("temporal_window_size", 20)));
    temporal_parameters.minimum_tracking_frames = static_cast<std::size_t>(
      std::max<std::int64_t>(1, declare_parameter<std::int64_t>("minimum_tracking_frames", 5)));
    temporal_parameters.maximum_failed_frames = static_cast<std::size_t>(
      std::max<std::int64_t>(1, declare_parameter<std::int64_t>("maximum_failed_frames", 5)));
    temporal_parameters.maximum_distance_jump_m = declare_parameter(
      "maximum_distance_jump_m", 0.03);
    temporal_parameters.maximum_normal_jump_deg = declare_parameter(
      "maximum_normal_jump_deg", 5.0);
    temporal_filter_ = std::make_unique<TemporalFilter>(temporal_parameters);

    normal_reference_frame_ = declare_parameter<std::string>(
      "normal_reference_frame", "base_link");
    expected_normal_reference_ = Eigen::Vector3d(
      declare_parameter("expected_normal_x", 0.0),
      declare_parameter("expected_normal_y", 0.0),
      declare_parameter("expected_normal_z", 1.0));
    tf_timeout_s_ = declare_parameter("tf_timeout_s", 0.10);
    allow_fixed_normal_fallback_ = declare_parameter(
      "allow_fixed_normal_fallback", false);
    fixed_normal_cloud_ = Eigen::Vector3d(
      declare_parameter("fixed_normal_cloud_x", 0.0),
      declare_parameter("fixed_normal_cloud_y", 0.0),
      declare_parameter("fixed_normal_cloud_z", -1.0));
    publish_inliers_ = declare_parameter("publish_inliers", false);
    publish_marker_ = declare_parameter("publish_marker", true);
    marker_size_m_ = declare_parameter("marker_size_m", 0.60);

    auto output_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    plane_publisher_ = create_publisher<msg::TablePlane>("~/plane", output_qos);
    distance_publisher_ = create_publisher<std_msgs::msg::Float64>(
      "~/distance", output_qos);
    optical_axis_publisher_ = create_publisher<std_msgs::msg::Float64>(
      "~/optical_axis_distance", output_qos);
    inliers_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "~/inliers", rclcpp::SensorDataQoS());
    marker_publisher_ = create_publisher<visualization_msgs::msg::Marker>(
      "~/marker", output_qos);
    diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(10).reliable());

    cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "~/input_cloud", rclcpp::SensorDataQoS(),
      std::bind(&TablePlaneDetectorNode::cloud_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "table plane detector ready: normal_reference_frame=%s, process_rate=%.2f Hz",
      normal_reference_frame_.c_str(), processing_rate_hz);
  }

private:
  bool expected_normal_in_cloud_frame(
    const std_msgs::msg::Header & header,
    Eigen::Vector3d & result,
    std::string & error)
  {
    if (expected_normal_reference_.norm() < 1.0e-9) {
      error = "configured expected normal has zero length";
      return false;
    }
    if (normal_reference_frame_.empty() || normal_reference_frame_ == header.frame_id) {
      result = expected_normal_reference_.normalized();
      return true;
    }
    try {
      const auto transform = tf_buffer_->lookupTransform(
        header.frame_id, normal_reference_frame_, rclcpp::Time(header.stamp),
        rclcpp::Duration::from_seconds(tf_timeout_s_));
      const auto & q = transform.transform.rotation;
      const Eigen::Quaterniond rotation(q.w, q.x, q.y, q.z);
      result = (rotation * expected_normal_reference_).normalized();
      return true;
    } catch (const tf2::TransformException & exception) {
      if (allow_fixed_normal_fallback_ && fixed_normal_cloud_.norm() > 1.0e-9) {
        result = fixed_normal_cloud_.normalized();
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "TF unavailable (%s); using explicit fixed normal fallback",
          exception.what());
        return true;
      }
      error = exception.what();
      return false;
    }
  }

  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_message)
  {
    const auto now = std::chrono::steady_clock::now();
    if (last_processing_time_.time_since_epoch().count() != 0 &&
      now - last_processing_time_ < minimum_processing_period_)
    {
      return;
    }
    last_processing_time_ = now;

    if (cloud_message->header.frame_id.empty()) {
      PlaneEstimate invalid;
      invalid.input_point_count = cloud_message->width * cloud_message->height;
      invalid.status = "EMPTY_FRAME_ID";
      publish_result(cloud_message->header, temporal_filter_->update(invalid), invalid);
      return;
    }

    Eigen::Vector3d expected_normal;
    std::string transform_error;
    if (!expected_normal_in_cloud_frame(
        cloud_message->header, expected_normal, transform_error))
    {
      PlaneEstimate invalid;
      invalid.input_point_count = cloud_message->width * cloud_message->height;
      invalid.status = "TF_UNAVAILABLE";
      publish_result(cloud_message->header, temporal_filter_->update(invalid), invalid);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "table normal TF unavailable: %s",
        transform_error.c_str());
      return;
    }

    auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    try {
      pcl::fromROSMsg(*cloud_message, *cloud);
    } catch (const std::exception & exception) {
      PlaneEstimate invalid;
      invalid.input_point_count = cloud_message->width * cloud_message->height;
      invalid.status = "INVALID_POINT_CLOUD";
      publish_result(cloud_message->header, temporal_filter_->update(invalid), invalid);
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "point cloud conversion failed: %s",
        exception.what());
      return;
    }

    const PlaneEstimate raw = estimator_->estimate(cloud, expected_normal);
    const PlaneEstimate stable = temporal_filter_->update(raw);
    publish_result(cloud_message->header, stable, raw);
  }

  void publish_result(
    const std_msgs::msg::Header & header,
    const PlaneEstimate & estimate,
    const PlaneEstimate & raw)
  {
    msg::TablePlane message;
    message.header = header;
    for (std::size_t index = 0; index < 4; ++index) {
      message.coefficients[index] = estimate.coefficients[static_cast<Eigen::Index>(index)];
    }
    message.perpendicular_distance_m = estimate.perpendicular_distance_m;
    message.optical_axis_distance_m = estimate.optical_axis_distance_m;
    message.input_point_count = static_cast<std::uint32_t>(estimate.input_point_count);
    message.filtered_point_count = static_cast<std::uint32_t>(estimate.filtered_point_count);
    message.inlier_count = static_cast<std::uint32_t>(estimate.inlier_count);
    message.inlier_ratio = estimate.inlier_ratio;
    message.rms_error_m = estimate.rms_error_m;
    message.normal_error_deg = estimate.normal_error_deg;
    message.valid = estimate.valid;
    message.status = estimate.status;
    plane_publisher_->publish(message);

    const double nan = std::numeric_limits<double>::quiet_NaN();
    std_msgs::msg::Float64 distance;
    distance.data = estimate.valid ? estimate.perpendicular_distance_m : nan;
    distance_publisher_->publish(distance);
    std_msgs::msg::Float64 optical_distance;
    optical_distance.data = estimate.valid && std::isfinite(estimate.optical_axis_distance_m) ?
      estimate.optical_axis_distance_m : nan;
    optical_axis_publisher_->publish(optical_distance);

    if (publish_inliers_ && !raw.inliers->empty()) {
      sensor_msgs::msg::PointCloud2 inlier_message;
      pcl::toROSMsg(*raw.inliers, inlier_message);
      inlier_message.header = header;
      inliers_publisher_->publish(inlier_message);
    }
    if (publish_marker_) {
      publish_marker(header, estimate);
    }
    publish_diagnostics(header, estimate);

    if (!estimate.valid) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "table plane invalid: %s (filtered=%zu, inliers=%zu, ratio=%.3f)",
        estimate.status.c_str(), estimate.filtered_point_count,
        estimate.inlier_count, estimate.inlier_ratio);
    }
  }

  void publish_marker(
    const std_msgs::msg::Header & header,
    const PlaneEstimate & estimate)
  {
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.ns = "table_plane";
    marker.id = 0;
    if (!estimate.valid) {
      marker.action = visualization_msgs::msg::Marker::DELETE;
      marker_publisher_->publish(marker);
      return;
    }
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    const Eigen::Vector3d normal = estimate.coefficients.head<3>().normalized();
    const Eigen::Vector3d point = -estimate.coefficients[3] * normal;
    const Eigen::Quaterniond orientation = Eigen::Quaterniond::FromTwoVectors(
      Eigen::Vector3d::UnitZ(), normal);
    marker.pose.position.x = point.x();
    marker.pose.position.y = point.y();
    marker.pose.position.z = point.z();
    marker.pose.orientation.x = orientation.x();
    marker.pose.orientation.y = orientation.y();
    marker.pose.orientation.z = orientation.z();
    marker.pose.orientation.w = orientation.w();
    marker.scale.x = marker_size_m_;
    marker.scale.y = marker_size_m_;
    marker.scale.z = 0.005;
    marker.color.r = 0.10F;
    marker.color.g = 0.85F;
    marker.color.b = 0.20F;
    marker.color.a = 0.45F;
    marker_publisher_->publish(marker);
  }

  void publish_diagnostics(
    const std_msgs::msg::Header & header,
    const PlaneEstimate & estimate)
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header = header;
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = get_fully_qualified_name() + std::string(": table plane");
    status.hardware_id = header.frame_id;
    status.level = estimate.valid ?
      diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = estimate.status;
    status.values.push_back(key_value("perpendicular_distance_m", estimate.perpendicular_distance_m));
    status.values.push_back(key_value("optical_axis_distance_m", estimate.optical_axis_distance_m));
    status.values.push_back(key_value("filtered_points", estimate.filtered_point_count));
    status.values.push_back(key_value("inliers", estimate.inlier_count));
    status.values.push_back(key_value("inlier_ratio", estimate.inlier_ratio));
    status.values.push_back(key_value("rms_error_m", estimate.rms_error_m));
    status.values.push_back(key_value("normal_error_deg", estimate.normal_error_deg));
    array.status.push_back(status);
    diagnostics_publisher_->publish(array);
  }

  std::unique_ptr<PlaneEstimator> estimator_;
  std::unique_ptr<TemporalFilter> temporal_filter_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  Eigen::Vector3d expected_normal_reference_;
  Eigen::Vector3d fixed_normal_cloud_;
  std::string normal_reference_frame_;
  double tf_timeout_s_{0.10};
  bool allow_fixed_normal_fallback_{false};
  bool publish_inliers_{false};
  bool publish_marker_{true};
  double marker_size_m_{0.60};

  std::chrono::duration<double> minimum_processing_period_{0.2};
  std::chrono::steady_clock::time_point last_processing_time_{};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
  rclcpp::Publisher<msg::TablePlane>::SharedPtr plane_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr distance_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr optical_axis_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr inliers_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
};

}  // namespace table_plane_detector

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<table_plane_detector::TablePlaneDetectorNode>());
  rclcpp::shutdown();
  return 0;
}

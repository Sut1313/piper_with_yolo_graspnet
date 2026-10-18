#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace table_plane_detector
{

class DepthImageToCloudNode : public rclcpp::Node
{
public:
  DepthImageToCloudNode()
  : Node("depth_image_to_cloud")
  {
    output_frame_ = declare_parameter<std::string>(
      "output_frame", "camera_depth_optical_frame");
    depth_scale_ = declare_parameter("depth_scale", 0.001);
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "~/points", rclcpp::SensorDataQoS());
    info_subscription_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "~/camera_info", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::CameraInfo::SharedPtr message) {
        camera_info_ = message;
      });
    depth_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      "~/depth", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Image::SharedPtr message) {
        convert(message);
      });
  }

private:
  float depth_at(const sensor_msgs::msg::Image & image, std::size_t offset) const
  {
    if (image.encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
      float value = std::numeric_limits<float>::quiet_NaN();
      std::memcpy(&value, image.data.data() + offset, sizeof(float));
      return value;
    }
    std::uint16_t value = 0;
    std::memcpy(&value, image.data.data() + offset, sizeof(std::uint16_t));
    return static_cast<float>(value * depth_scale_);
  }

  void convert(const sensor_msgs::msg::Image::SharedPtr & image)
  {
    if (!camera_info_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "waiting for CameraInfo");
      return;
    }
    const bool float_depth =
      image->encoding == sensor_msgs::image_encodings::TYPE_32FC1;
    const bool integer_depth =
      image->encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
      image->encoding == sensor_msgs::image_encodings::MONO16;
    if (!float_depth && !integer_depth) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "unsupported depth encoding: %s",
        image->encoding.c_str());
      return;
    }

    const double fx = camera_info_->k[0];
    const double fy = camera_info_->k[4];
    const double cx = camera_info_->k[2];
    const double cy = camera_info_->k[5];
    if (fx <= 0.0 || fy <= 0.0) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "invalid camera intrinsics");
      return;
    }

    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.width = image->width;
    cloud.height = image->height;
    cloud.is_dense = false;
    cloud.resize(static_cast<std::size_t>(cloud.width) * cloud.height);
    const std::size_t pixel_size = float_depth ? sizeof(float) : sizeof(std::uint16_t);
    const float nan = std::numeric_limits<float>::quiet_NaN();
    for (std::uint32_t row = 0; row < image->height; ++row) {
      for (std::uint32_t column = 0; column < image->width; ++column) {
        const std::size_t data_offset =
          static_cast<std::size_t>(row) * image->step + column * pixel_size;
        const float z = depth_at(*image, data_offset);
        auto & point = cloud.at(column, row);
        if (!std::isfinite(z) || z <= 0.0F) {
          point.x = nan;
          point.y = nan;
          point.z = nan;
          continue;
        }
        point.z = z;
        point.x = static_cast<float>((static_cast<double>(column) - cx) * z / fx);
        point.y = static_cast<float>((static_cast<double>(row) - cy) * z / fy);
      }
    }

    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(cloud, output);
    output.header = image->header;
    output.header.frame_id = output_frame_.empty() ? image->header.frame_id : output_frame_;
    publisher_->publish(output);
  }

  std::string output_frame_;
  double depth_scale_{0.001};
  sensor_msgs::msg::CameraInfo::SharedPtr camera_info_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

}  // namespace table_plane_detector

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<table_plane_detector::DepthImageToCloudNode>());
  rclcpp::shutdown();
  return 0;
}

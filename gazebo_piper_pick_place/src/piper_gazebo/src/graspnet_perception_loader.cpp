#include "piper_gazebo/pick_node.hpp"

#include <ctime>
#include <fstream>
#include <sstream>
#include <sys/stat.h>

#include <rclcpp/rclcpp.hpp>

namespace piper_gazebo::pick_place {

std::vector<std::string> PickNode::splitCsvLine(const std::string& line) {
  std::vector<std::string> fields;
  std::stringstream ss(line);
  std::string field;
  while (std::getline(ss, field, ',')) {
    fields.push_back(field);
  }
  return fields;
}

bool PickNode::loadPerceptionTargets(
    std::map<std::string, std::vector<PerceptionTarget>>& targets) {
  const std::string path = get_parameter_or<std::string>(
      "perception_grasps_path",
      "shared_camera_data/perception_grasps.csv");

  struct stat file_stat
  {
  };
  if (stat(path.c_str(), &file_stat) != 0) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000, "Cannot stat perception grasps CSV: %s",
                          path.c_str());
    return false;
  }
  const double age = std::difftime(std::time(nullptr), file_stat.st_mtime);
  if (age > kPerceptionMaxAgeSec) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000,
                          "Perception grasps CSV is stale: age %.1fs > %.1fs", age,
                          kPerceptionMaxAgeSec);
    return false;
  }

  std::ifstream file(path);
  if (!file.is_open()) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000, "Cannot open perception grasps CSV: %s",
                          path.c_str());
    return false;
  }

  std::string header;
  std::getline(file, header);
  std::string line;
  while (std::getline(file, line)) {
    if (line.empty()) {
      continue;
    }
    const auto fields = splitCsvLine(line);
    if (fields.size() < 23) {
      RCLCPP_WARN(get_logger(), "Malformed perception grasp row: %s", line.c_str());
      continue;
    }

    PerceptionTarget target;
    try {
      target.camera = {std::stod(fields[1]), std::stod(fields[2]), std::stod(fields[3])};
      target.world = cameraToWorld(target.camera);
      target.count = std::stoi(fields[4]);
      target.score = std::stod(fields[5]);
      target.width = std::stod(fields[6]);
      target.height = std::stod(fields[7]);
      target.depth = std::stod(fields[8]);
      for (int i = 0; i < 9; ++i) {
        target.camera_rotation[i] = std::stod(fields[9 + i]);
      }
      target.object_center_camera = {std::stod(fields[18]), std::stod(fields[19]),
                                     std::stod(fields[20])};
      target.object_center_world = cameraToWorld(target.object_center_camera);
      target.link6_grasp_z = std::stod(fields[21]);
      target.tcp_forward_z = std::stod(fields[22]);
      target.orientation = cameraGraspRotationToWorldTcp(target.camera_rotation);
      target.has_grasp_pose = true;
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(), "Failed to parse row '%s': %s", line.c_str(), e.what());
      continue;
    }
    targets[fields[0]].push_back(target);
  }

  if (targets.empty()) {
    RCLCPP_ERROR(get_logger(), "Perception grasps CSV contains no targets.");
    return false;
  }
  return true;
}

} // namespace piper_gazebo::pick_place

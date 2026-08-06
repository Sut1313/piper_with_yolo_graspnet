#include "piper_gazebo/pick_node.hpp"

#include <sys/stat.h>

#include <fstream>
#include <sstream>

#include <rclcpp/rclcpp.hpp>

namespace piper_gazebo::pick_place {

bool PickNode::writePickStateFile(const std::set<std::string>& placed_objects) {
  // shared_dir 参数（launch 已传）；为空时从 perception_grasps_path 父目录推导
  std::string dir = get_parameter_or<std::string>("shared_dir", "");
  if (dir.empty()) {
    const std::string csv = get_parameter_or<std::string>("perception_grasps_path", "");
    const auto pos = csv.find_last_of('/');
    dir = pos == std::string::npos ? "." : csv.substr(0, pos);
  }
  ::mkdir(dir.c_str(), 0755); // 父目录必存在，EEXIST 忽略

  const std::string path = dir + "/pick_state.json";
  const std::string tmp = path + ".tmp";

  // JSON 拼装（固定 schema，手拼；名称做防御性转义）
  auto escape = [](const std::string& s) {
    std::string out;
    for (char c : s) {
      if (c == '"' || c == '\\') {
        out += '\\';
      }
      out += c;
    }
    return out;
  };
  std::ostringstream oss;
  oss << "{\"seq\": " << pick_state_seq_++ << ", \"pending\": [";
  bool first = true;
  for (const auto& obj : objects()) {
    if (placed_objects.count(obj.name) > 0) {
      continue;
    }
    if (!first) {
      oss << ", ";
    }
    oss << '"' << escape(perceptionKeyForObject(obj.name)) << '"';
    first = false;
  }
  oss << "], \"placed\": [";
  first = true;
  for (const auto& name : placed_objects) { // std::set 有序，输出确定
    if (!first) {
      oss << ", ";
    }
    oss << '"' << escape(perceptionKeyForObject(name)) << '"';
    first = false;
  }
  oss << "]}\n";

  // 原子写：tmp + rename（同目录 POSIX 原子，Docker 读无撕裂）
  {
    std::ofstream out(tmp);
    if (!out) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Cannot write pick state tmp: %s",
                            tmp.c_str());
      return false;
    }
    out << oss.str();
  }
  if (std::rename(tmp.c_str(), path.c_str()) != 0) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Cannot rename pick state: %s",
                          tmp.c_str());
    return false;
  }
  RCLCPP_INFO(get_logger(), "pick_state.json written: seq=%d, placed=%zu", pick_state_seq_,
              placed_objects.size());
  return true;
}

} // namespace piper_gazebo::pick_place

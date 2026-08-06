#pragma once

#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>

namespace piper_gazebo::pick_place {

struct PiperAnalyticIkSolution
{
  std::vector<double> joints;
  double residual{0.0};
  double seed_cost{0.0};
};

std::vector<PiperAnalyticIkSolution>
solvePiperAnalyticIkForTcp(const geometry_msgs::msg::Pose& target_tcp_pose,
                           const std::vector<double>& current_joints, int max_solutions,
                           const std::string& script_path);

} // namespace piper_gazebo::pick_place

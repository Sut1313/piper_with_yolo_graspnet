#include "piper_gazebo/grasp_planners.hpp"

#include <algorithm>
#include <array>

#include <rclcpp/rclcpp.hpp>

#include "piper_gazebo/motion_planning.hpp"
#include "piper_gazebo/pick_node.hpp"

namespace piper_gazebo::pick_place {

bool chooseReachablePositionGraspCandidate(
    PickNode& node, moveit::planning_interface::MoveGroupInterface& move_group,
    const Object& object_template, double approach_clearance, double lift_z, double grasp_z_offset,
    double min_grasp_z, double tool_z_offset, double open_gripper_lift_height, double back_place_x,
    double back_place_y, const geometry_msgs::msg::Quaternion& back_place_orientation,
    Object& selected_object, geometry_msgs::msg::Quaternion& selected_orientation) {
  geometry_msgs::msg::Point visual_center;
  if (!node.readValidatedVisionObjectPosition(object_template.name, visual_center)) {
    return false;
  }

  Object object = object_template;
  object.x = visual_center.x;
  object.y = visual_center.y;
  object.z = std::max(visual_center.z, kMinReachableTcpGraspZ);

  const std::array<OrientationVariant, 2> tcp_orientations{{
      {"top_down_position_only", topDownTcpOrientation()},
      {"home_position_only", back_place_orientation},
  }};

  for (size_t orient_idx = 0; orient_idx < tcp_orientations.size(); ++orient_idx) {
    const auto& tcp_orientation = tcp_orientations[orient_idx].orientation;
    const auto poses = makeObjectPoses(object, tcp_orientation, approach_clearance, lift_z,
                                       grasp_z_offset, min_grasp_z, 0.0, tool_z_offset, false);
    RCLCPP_INFO(node.get_logger(),
                "%s position-only candidate: center=(%.4f, %.4f, %.4f), "
                "orientation=%s, approach=(%.4f, %.4f, %.4f)",
                object.name.c_str(), object.x, object.y, object.z,
                tcp_orientations[orient_idx].name.c_str(), poses.approach.position.x,
                poses.approach.position.y, poses.approach.position.z);

    if (!canPlanFullPose(node, move_group, poses.grasp, object.name + " position-only grasp")) {
      continue;
    }

    auto open_lift_pose = poses.grasp;
    open_lift_pose.position.z += open_gripper_lift_height;
    if (!canPlanFullPose(node, move_group, open_lift_pose, object.name + " position-only lift")) {
      continue;
    }

    auto back_place_pose = open_lift_pose;
    back_place_pose.position.x = back_place_x;
    back_place_pose.position.y = back_place_y;
    back_place_pose.orientation = back_place_orientation;
    if (!canPlanFullPose(node, move_group, back_place_pose,
                         object.name + " position-only place target")) {
      continue;
    }

    selected_object = object;
    selected_orientation = tcp_orientation;
    RCLCPP_WARN(node.get_logger(),
                "%s selected POSITION-ONLY baseline candidate with orientation=%s. "
                "This path ignores GraspNet grasp orientation and is for fallback/debug.",
                object.name.c_str(), tcp_orientations[orient_idx].name.c_str());
    return true;
  }

  RCLCPP_ERROR(node.get_logger(),
               "No position-only candidate for %s has a reachable grasp, lift, "
               "and place transfer target.",
               object_template.name.c_str());
  return false;
}

} // namespace piper_gazebo::pick_place

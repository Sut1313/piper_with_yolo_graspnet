#pragma once

// moveit 的 urdf/urdf/model.h 需要完整类型定义（必须位于 moveit include 之前）
#include <urdfdom_headers/urdf_model/model.h>

#include <moveit/move_group_interface/move_group_interface.h>

#include "piper_gazebo/grasp_common.hpp"

// ============================================================================
// 抓取方案选择器接口声明
//   chooseReachableGraspCandidate         —— GraspNet 6D 候选遍历（4 段预检），
//                                            实现见 src/graspnet_6d_grasp_planner.cpp
//   chooseReachablePositionGraspCandidate —— 仅位置回退抓取（俯视姿态），
//                                            实现见 src/position_grasp_planner.cpp
//   两者输出一致：选中的物体参数（x/y/z 为抓取 TCP 位置）与 TCP 姿态。
//   6D 主路径全部候选不可达时，由调用方（pickObject）决定是否启用位置回退
// ============================================================================
namespace piper_gazebo::pick_place {

class PickNode;

// 位置回退：忽略 GraspNet 方向预测，用俯视姿态（topDownTcpOrientation）
// 挑选可达抓取点（仅位置信息可信时的兜底路径）
bool chooseReachablePositionGraspCandidate(
    PickNode& node, moveit::planning_interface::MoveGroupInterface& move_group,
    const Object& object_template, double approach_clearance, double lift_z, double grasp_z_offset,
    double min_grasp_z, double tool_z_offset, double open_gripper_lift_height, double back_place_x,
    double back_place_y, const geometry_msgs::msg::Quaternion& back_place_orientation,
    Object& selected_object, geometry_msgs::msg::Quaternion& selected_orientation);

// 6D 主路径：遍历 GraspNet 候选，4 段预检（approach→grasp→lift→place）
// 全部通过才选中，保留 GraspNet 预测的完整抓取姿态
bool chooseReachableGraspCandidate(
    PickNode& node, moveit::planning_interface::MoveGroupInterface& move_group,
    const Object& object_template, double approach_clearance, double lift_z, double grasp_z_offset,
    double min_grasp_z, double tool_z_offset, double open_gripper_lift_height, double back_place_x,
    double back_place_y, const geometry_msgs::msg::Quaternion& back_place_orientation,
    Object& selected_object, geometry_msgs::msg::Quaternion& selected_orientation);

} // namespace piper_gazebo::pick_place

#pragma once

#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
// moveit 的 urdf/urdf/model.h 需要完整类型定义（必须位于 moveit include 之前）
#include <urdfdom_headers/urdf_model/model.h>

#include <moveit/move_group_interface/move_group_interface.h>

#include "piper_gazebo/grasp_common.hpp"

// ============================================================================
// 运动规划接口声明（实现位于 src/pick_and_place.cpp）
//
//   规划策略分三层，失败逐级降级：
//     1. moveLinear              —— 笛卡尔直线规划（优先，直线轨迹利于避碰），
//                                  覆盖率不足 0.9 时降级到第 2 层
//     2. planFullPoseWithSeeds   —— ssik 解析 IK 种子 + 关节空间规划
//     3. moveJoints              —— 纯关节角目标（回家/零位，无位姿约束）
//   候选预检（canPlanFullPose）只规划不执行，供 4 段预检流水线使用
// ============================================================================
namespace piper_gazebo::pick_place {

class PickNode;

// 以 ssik 解析 IK 解为种子规划目标 TCP 位姿（含 FK 回验；预检/执行共用）
bool planFullPoseWithSeeds(PickNode& node,
                           moveit::planning_interface::MoveGroupInterface& move_group,
                           const geometry_msgs::msg::Pose& target, const std::string& label,
                           double planning_time,
                           moveit::planning_interface::MoveGroupInterface::Plan& plan);

// 候选预检（dry-run）：只规划不执行，判断目标位姿是否可达
bool canPlanFullPose(PickNode& node, moveit::planning_interface::MoveGroupInterface& move_group,
                     const geometry_msgs::msg::Pose& target, const std::string& label);

// 笛卡尔直线运动：computeCartesianPath 优先，覆盖率不足时降级为关节空间规划
bool moveLinear(PickNode& node, moveit::planning_interface::MoveGroupInterface& move_group,
                const geometry_msgs::msg::Pose& target, const std::string& label);

// 关节空间运动：直接指定关节角目标（回家/零位），不做位姿约束
bool moveJoints(PickNode& node, moveit::planning_interface::MoveGroupInterface& move_group,
                const std::vector<double>& joints, const std::string& label);

// 统一配置 MoveGroupInterface：末端 link/参考系/规划时间/容差/速度缩放
void configureMoveGroup(moveit::planning_interface::MoveGroupInterface& move_group);

// 读取当前 TCP 世界坐标（tool_z_offset 为兼容参数，未使用）
geometry_msgs::msg::Point currentTcpPosition(
    moveit::planning_interface::MoveGroupInterface& move_group, double tool_z_offset);

// 距离门控校验：TCP 与物体（竖直圆柱）中心距 ≤ max_distance（默认 0.08m）
bool verifyCurrentTcpNearObject(PickNode& node,
                                moveit::planning_interface::MoveGroupInterface& move_group,
                                const std::string& object_name, double tool_z_offset,
                                double max_distance, const std::string& label);

} // namespace piper_gazebo::pick_place

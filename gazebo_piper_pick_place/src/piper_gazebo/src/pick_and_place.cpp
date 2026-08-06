/**
 * ============================================================================
 *  piper_gazebo/src/pick_and_place.cpp
 *
 *  【文件概述】
 *  Piper 六轴机械臂的自动抓取（Pick-and-Place）主程序。
 *  通过 MoveIt2 的规划接口、夹爪控制和 Gazebo 实时状态读取，
 *  实现对多种形状物体的全自动流水线式抓取与码放。
 *
 *  【执行的数据流】
 *  1. main() 启动 PickNode（ROS2 节点），等待运行时依赖就绪
 *  2. 连接到 MoveIt2 MoveGroupInterface — 运动规划 / 执行
 *  3. 机械臂回到 home 关节姿态
 *  4. 对每个目标物体（green_cylinder / red_box / blue_sphere，顺序可随机化）：
 *     a. 读取感知缓存中的 GraspNet 候选，按视觉中心距离升序 +
 *        round-robin 姿态桶重排（见 graspnet_6d_grasp_planner.cpp）
 *     b. 4 段预检（AND）：approach→grasp→lift→place 逐段 dry-run 规划，
 *        全部可达才执行；无任何可达候选时按 enable_position_only_fallback
 *        决定是回退到仅位置抓取（默认）还是中止主循环
 *     c. 执行步骤：approach → descent → attach（距离门控）
 *        → open-gripper lift → place transfer → detach
 *     d. 优先使用笛卡尔直线规划（computeCartesianPath），
 *        失败时降级为 ssik 解析 IK 关节空间规划（planFullPoseWithSeeds）
 *     e. 抓取成功则记录（写 pick_state.json），全部放完后回到零位
 *  5. 关机 — 返回码 0=全部成功, 1=部分失败, 2/3=启动失败
 *
 *  【关键依赖】
 *    - /gripper_controller/follow_joint_trajectory (Action) — 夹爪控制
 *    - /get_entity_state (Service) — Gazebo 物体位置读取
 *    - /get_entity_state + MoveIt 当前末端位姿 — 抓取距离实时校验
 *    - /move_action (Action) — MoveIt2 move_group 运动规划
 * ============================================================================
 */

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <ctime>
#include <fstream>
#include <future>
#include <iterator>
#include <map>
#include <random>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include <cstdio>
#include <sys/stat.h>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <gazebo_msgs/srv/get_entity_state.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <urdfdom_headers/urdf_model/model.h>
#include "piper_gazebo/grasp_common.hpp"
#include "piper_gazebo/grasp_planners.hpp"
#include "piper_gazebo/motion_planning.hpp"
#include "piper_gazebo/pick_node.hpp"
#include "piper_gazebo/piper_analytic_ik.hpp"
#include "piper_gazebo/srv/attach_detach.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/action/move_group.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

using namespace std::chrono_literals;

// ============================================================================
//  共享抓取类型、PickNode 接口和运动规划实现
// ============================================================================

namespace piper_gazebo::pick_place {

// ============================================================================
//  全局辅助函数（独立于 PickNode）
// ============================================================================

// Forward declaration
bool planFullPoseWithSeeds(PickNode& node,
                           moveit::planning_interface::MoveGroupInterface& move_group,
                           const geometry_msgs::msg::Pose& target, const std::string& label,
                           double planning_time,
                           moveit::planning_interface::MoveGroupInterface::Plan& plan);

/**
 * moveLinear() — 笛卡尔直线运动（优先） + 关节空间降级
 *
 *  优先尝试 computeCartesianPath() 规划直线轨迹，因为抓取码放要求
 *  末端执行器沿直线运动以避碰。如果直线规划覆盖率不足 90%，
 *  则降级为 setPoseTarget() 做普通关节空间规划。
 *
 *  【为何降级】
 *    笛卡尔规划在奇异点附近或长距离移动时可能失败，降级到关节空间
 *    可以保证任务继续执行，尽管路径不再是直线。
 *
 *  @param node        PickNode 引用（用于日志）
 *  @param move_group  MoveIt MoveGroupInterface 引用
 *  @param target      目标位姿
 *  @param label       步骤名称（日志输出用）
 *  @return true=运动执行成功
 */
bool moveLinear(PickNode& node, moveit::planning_interface::MoveGroupInterface& move_group,
                const geometry_msgs::msg::Pose& target, const std::string& label) {
  // 规划执行前等待 0.5s：让 joint_states/执行器状态刷新到最新，
  // 避免轨迹起点校验失败导致 execute() failed or timeout
  rclcpp::sleep_for(500ms);
  // 记录当前起点（仅用于降级警告日志输出）
  const auto start = move_group.getCurrentPose(kEndEffectorLink).pose.position;

  std::vector<geometry_msgs::msg::Pose> waypoints{target};
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double fraction =
      move_group.computeCartesianPath(waypoints, kCartesianStep, 0.0, trajectory);

  if (fraction >= kCartesianMinFraction) {
    // ── 笛卡尔规划成功，直接执行 ──
    const bool executed = static_cast<bool>(move_group.execute(trajectory));
    if (!executed) {
      RCLCPP_ERROR(node.get_logger(), "Cartesian execution failed for %s.", label.c_str());
      return false;
    }

    rclcpp::sleep_for(300ms);
    return true;
  }

  // ── 笛卡尔规划覆盖率不足，降级到带种子的关节空间规划 ──
  RCLCPP_WARN(node.get_logger(),
              "Cartesian planning incomplete for %s start=(%.3f, %.3f, %.3f) "
              "target=(%.3f, %.3f, %.3f), fraction=%.3f. "
              "Falling back to seeded full-pose planning.",
              label.c_str(), start.x, start.y, start.z, target.position.x, target.position.y,
              target.position.z, fraction);

  // 降级规划使用候选预检的短规划时间（kCandidatePlanningTime=5s），
  // 不占用正式执行的长规划预算
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (!planFullPoseWithSeeds(node, move_group, target, label, kCandidatePlanningTime, plan)) {
    RCLCPP_ERROR(node.get_logger(), "Seeded full-pose planning fallback failed for %s.",
                 label.c_str());
    return false;
  }

  const bool executed = static_cast<bool>(move_group.execute(plan));
  if (!executed) {
    RCLCPP_ERROR(node.get_logger(), "Seeded full-pose planning fallback execution failed for %s.",
                 label.c_str());
    return false;
  }

  rclcpp::sleep_for(300ms);
  return true;
}

/**
 * planFullPoseWithSeeds() — 以 ssik 解析 IK 求解结果作为关节种子，
 * 对目标 TCP 位姿做完整关节空间规划（预检 dry-run 与正式执行共用）。
 *
 *  流程：
 *    1. 读取当前关节角作为种子
 *    2. 调用 solvePiperAnalyticIkForTcp 求 link6 解析解列表
 *    3. 逐解尝试：MoveIt FK 正解回验 TCP 位置/姿态误差
 *       （位置 >1.5cm 或角度 >0.1rad 直接丢弃 —— 解析解与 URDF 建模
 *       不一致时宁可不规划，防止轨迹终点偏离目标）
 *    4. 通过回验的解经 setJointValueTarget + plan() 交给 MoveIt 规划
 *       （碰撞与轨迹由 MoveIt 保证，本层只替代 IK 采样阶段）
 *    5. 全部失败后按 graspnet_6d_analytic_only_precheck 决定是否
 *       KDL/direct 兜底：预检阶段默认禁止 —— ssik 失败即真不可达，
 *       兜底由候选遍历（graspnet_6d_max_candidates）提供
 *
 *  @return true=找到可行规划；false=失败（调用方决定候选是否被跳过）
 */
bool planFullPoseWithSeeds(PickNode& node,
                           moveit::planning_interface::MoveGroupInterface& move_group,
                           const geometry_msgs::msg::Pose& target, const std::string& label,
                           double planning_time,
                           moveit::planning_interface::MoveGroupInterface::Plan& plan) {
  const double previous_planning_time = move_group.getPlanningTime();
  // 预检/快速求解用 1s 上限的短规划时间（只探可达性），
  // 结束后恢复调用前的规划时间（正式执行用 kExecutionPlanningTime）
  move_group.setPlanningTime(std::min(planning_time, 1.0));

  const bool enable_piper_analytic_ik =
      node.get_parameter_or<bool>("enable_piper_analytic_ik_planning", true);
  if (enable_piper_analytic_ik) {
    // 读取当前状态作为种子（读取失败则跳过解析 IK 层，走 analytic-only 门控）
    auto current_state = move_group.getCurrentState(1.0);
    if (!current_state) {
      RCLCPP_WARN(node.get_logger(),
                  "Cannot get current robot state for Piper analytic IK while planning %s.",
                  label.c_str());
    } else {
      const auto* joint_model_group = current_state->getJointModelGroup(kMoveGroupName);
      if (joint_model_group == nullptr) {
        RCLCPP_ERROR(node.get_logger(), "Cannot find joint model group %s while planning %s.",
                     kMoveGroupName, label.c_str());
      } else {
        std::vector<double> current_joints;
        current_state->copyJointGroupPositions(joint_model_group, current_joints);

        const int max_analytic_solutions =
            std::max(1, node.get_parameter_or<int>("piper_analytic_ik_max_solutions", 8));
        const int max_analytic_solution_plans =
            std::max(1, node.get_parameter_or<int>("piper_analytic_ik_max_solution_plans", 6));
        const std::string analytic_script_path =
            node.get_parameter_or<std::string>("piper_analytic_ik_script", "");

        // 解析 IK 脚本异常仅告警不阻断：ssik 不可用时按 analytic-only 门控
        // 直接返回失败，由候选遍历继续尝试下一候选
        std::vector<PiperAnalyticIkSolution> analytic_solutions;
        try {
          analytic_solutions = solvePiperAnalyticIkForTcp(
              target, current_joints, max_analytic_solutions, analytic_script_path);
        } catch (const std::exception& ex) {
          RCLCPP_WARN(node.get_logger(), "Piper analytic IK helper failed for %s: %s",
                      label.c_str(), ex.what());
        }

        // 逐解尝试：先 FK 回验（解析解可能因 URDF 偏差/脚本精度有残差），
        // 通过后再交给 MoveIt 规划；最多试 max_analytic_solution_plans 个解
        size_t planned_attempts = 0;
        for (const auto& solution : analytic_solutions) {
          if (planned_attempts++ >= static_cast<size_t>(max_analytic_solution_plans)) {
            break;
          }

          auto target_state = std::make_shared<moveit::core::RobotState>(*current_state);
          target_state->setJointGroupPositions(joint_model_group, solution.joints);
          target_state->enforceBounds(joint_model_group);

          const Eigen::Isometry3d ik_eigen = target_state->getGlobalLinkTransform(kEndEffectorLink);
          const double dx = ik_eigen.translation().x() - target.position.x;
          const double dy = ik_eigen.translation().y() - target.position.y;
          const double dz = ik_eigen.translation().z() - target.position.z;
          const double pos_err = std::sqrt(dx * dx + dy * dy + dz * dz);
          Eigen::Quaterniond q_actual(ik_eigen.rotation());
          Eigen::Quaterniond q_target(target.orientation.w, target.orientation.x,
                                      target.orientation.y, target.orientation.z);
          q_target.normalize();
          const double dot = std::abs(q_actual.dot(q_target));
          const double angle_err = dot < 1.0 ? 2.0 * std::acos(std::clamp(dot, -1.0, 1.0)) : 0.0;
          // FK 回验门限：位置误差 >1.5cm 或角度误差 >0.1rad 的解不可信，
          // 丢弃（宁可不规划，防止轨迹终点偏离目标位姿）
          if (pos_err > 0.015 || angle_err > 0.10) {
            RCLCPP_WARN(node.get_logger(),
                        "Piper analytic IK rejected by MoveIt FK for %s: "
                        "pos_err=%.4f angle_err=%.3f residual=%.3g cost=%.3f.",
                        label.c_str(), pos_err, angle_err, solution.residual, solution.seed_cost);
            continue;
          }

          // 以当前状态为起点，把解析解直接设为关节目标，交给 MoveIt 做
          // 碰撞感知的轨迹规划（本层只替代 IK 采样阶段）
          move_group.setStartStateToCurrentState();
          if (!move_group.setJointValueTarget(*target_state)) {
            continue;
          }
          if (static_cast<bool>(move_group.plan(plan))) {
            RCLCPP_INFO(node.get_logger(),
                        "Piper analytic IK planning PASS for %s "
                        "(solutions=%zu, planned_attempt=%zu, residual=%.3g, cost=%.3f).",
                        label.c_str(), analytic_solutions.size(), planned_attempts,
                        solution.residual, solution.seed_cost);
            move_group.setPlanningTime(previous_planning_time);
            return true;
          }
        }

        RCLCPP_WARN(node.get_logger(),
                    "Piper analytic IK planning failed for %s: "
                    "solutions=%zu, planned_attempts=%zu.",
                    label.c_str(), analytic_solutions.size(), planned_attempts);
      }
    }
  }

  // analytic-only 门控：预检阶段（visual-center 标签）禁止 KDL/direct 兜底 ——
  // ssik 解析 IK 失败即真不可达，由候选遍历提供回退（防止数值 IK 的假阳性解）
  if (label.find("visual-center") != std::string::npos &&
      node.get_parameter_or<bool>("graspnet_6d_analytic_only_precheck", true)) {
    move_group.setPlanningTime(previous_planning_time);
    RCLCPP_WARN(node.get_logger(),
                "Skipping KDL/direct fallback for %s because GraspNet 6D "
                "candidate precheck is analytic-only.",
                label.c_str());
    return false;
  }

  // KDL 直接位姿规划层已删除：ssik 解析 IK 失败即为最终失败，
  // 兜底由候选遍历（graspnet_6d_max_candidates）提供。
  move_group.setPlanningTime(previous_planning_time);
  RCLCPP_WARN(node.get_logger(),
              "Full-pose planning failed for %s after analytic IK; "
              "ssik failure is final, candidate traversal provides the fallback.",
              label.c_str());
  return false;
}
// 候选预检（dry-run）：只规划不执行，验证目标位姿"是否可达"。
// 供 GraspNet 候选遍历的 4 段预检流水线调用（approach→grasp→lift→place），
// 以及 pickObject 执行前的最后 2 段预检（open-gripper lift / place target）
bool canPlanFullPose(PickNode& node, moveit::planning_interface::MoveGroupInterface& move_group,
                     const geometry_msgs::msg::Pose& target, const std::string& label) {
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const bool planned =
      planFullPoseWithSeeds(node, move_group, target, label, kCandidatePlanningTime, plan);
  if (!planned) {
    RCLCPP_WARN(node.get_logger(), "Candidate dry-run full-pose planning failed for %s.",
                label.c_str());
    return false;
  }
  RCLCPP_INFO(node.get_logger(), "Candidate dry-run full-pose planning PASS for %s.",
              label.c_str());
  return true;
}

/**
 * moveJoints() — 关节空间运动（直接设置关节角度目标）
 *
 *  用于回到 home 姿态或最终零位等不需要直线约束的动作。
 *
 *  @param node        PickNode 引用（日志用）
 *  @param move_group  MoveIt MoveGroupInterface 引用
 *  @param joints      目标关节角度向量（6 个 joint）
 *  @param label       动作名称（日志用）
 *  @return true=规划并执行成功
 */
bool moveJoints(PickNode& node, moveit::planning_interface::MoveGroupInterface& move_group,
                const std::vector<double>& joints, const std::string& label) {
  // 规划执行前等待 0.5s：让 joint_states/执行器状态刷新到最新，
  // 避免 execute() 用陈旧起点做轨迹起点校验而 failed or timeout
  rclcpp::sleep_for(500ms);
  move_group.setJointValueTarget(joints);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const bool planned = static_cast<bool>(move_group.plan(plan));
  if (!planned) {
    RCLCPP_WARN(node.get_logger(), "Joint planning failed for %s.", label.c_str());
    return false;
  }

  const bool executed = static_cast<bool>(move_group.execute(plan));
  if (!executed) {
    RCLCPP_WARN(node.get_logger(), "Joint execution failed for %s.", label.c_str());
  }
  return executed;
}

/**
 * configureMoveGroup() — 配置 MoveIt MoveGroupInterface 的通用参数
 *
 *  设置末端执行器 Link、参考坐标系、规划时间和容差。
 *  降速以保障 Gazebo 仿真中的稳定性。
 */
void configureMoveGroup(moveit::planning_interface::MoveGroupInterface& move_group) {
  move_group.setEndEffectorLink(kEndEffectorLink);
  move_group.setPoseReferenceFrame("world");
  move_group.setPlanningTime(kExecutionPlanningTime);
  move_group.setGoalPositionTolerance(0.02);
  move_group.setGoalOrientationTolerance(0.05);
  move_group.setMaxVelocityScalingFactor(0.35);
  move_group.setMaxAccelerationScalingFactor(0.35);
}

/**
 * waitForMoveGroupAction() — 等待 MoveIt move_group 的 Action Server 就绪
 *
 *  检查 /move_action Action Server 是否可用。
 *  这是 move_group 节点提供的核心运动规划接口。
 *
 *  @param node  ROS2 节点共享指针（用于创建 Action Client）
 *  @return true=8 秒内就绪
 */
bool waitForMoveGroupAction(const rclcpp::Node::SharedPtr& node) {
  auto move_action_client =
      rclcpp_action::create_client<moveit_msgs::action::MoveGroup>(node, "/move_action");
  if (move_action_client->wait_for_action_server(8s)) {
    return true;
  }

  RCLCPP_ERROR(node->get_logger(), "Missing MoveIt action server /move_action. "
                                   "Make sure move_group is running.");
  return false;
}

geometry_msgs::msg::Point currentTcpPosition(
    moveit::planning_interface::MoveGroupInterface& move_group, double tool_z_offset) {
  (void)tool_z_offset;
  return move_group.getCurrentPose(kEndEffectorLink).pose.position;
}

/**
 * verifyCurrentTcpNearObject() — TCP 与物体距离门控（默认 0.08m）
 *
 *  门控检查链的一部分，在三个关键时点调用：
 *    - attach 前：距离过远说明抓取点不对，拒绝 attach（防抓空）
 *    - 抬升后：距离过远说明物体没被带上（attach 未生效）
 *    - 转移后：距离过远说明物体中途脱落
 *  距离以"点到竖直圆柱表面"计算，比纯欧氏距离更贴近物理接触。
 *
 *  @return true=距离在门控内；false=超限（调用方执行清理/中止）
 */
bool verifyCurrentTcpNearObject(PickNode& node,
                                moveit::planning_interface::MoveGroupInterface& move_group,
                                const std::string& object_name, double tool_z_offset,
                                double max_distance, const std::string& label) {
  geometry_msgs::msg::Point object_position;
  if (!node.readObjectPosition(object_name, object_position)) {
    RCLCPP_ERROR(node.get_logger(),
                 "%s distance gate failed for %s: cannot read Gazebo object pose.", label.c_str(),
                 object_name.c_str());
    return false;
  }

  (void)tool_z_offset;
  const auto tcp_pose = move_group.getCurrentPose(kEndEffectorLink).pose;
  const auto tcp = tcp_pose.position;
  const double dx = object_position.x - tcp.x;
  const double dy = object_position.y - tcp.y;
  const double dz = object_position.z - tcp.z;
  const auto distance = distanceToVerticalCylinder(tcp, object_position);
  const double dist = distance.center;

  if (dist > max_distance) {
    RCLCPP_ERROR(node.get_logger(),
                 "%s distance gate FAIL for %s: tcp=(%.4f, %.4f, %.4f) "
                 "object=(%.4f, %.4f, %.4f) delta=(%.4f, %.4f, %.4f) "
                 "center_dist=%.4f m > %.4f m, surface_dist=%.4f m. Refusing forced grasp.",
                 label.c_str(), object_name.c_str(), tcp.x, tcp.y, tcp.z, object_position.x,
                 object_position.y, object_position.z, dx, dy, dz, distance.center, max_distance,
                 distance.surface);
    return false;
  }

  RCLCPP_INFO(node.get_logger(),
              "%s contact gate PASS for %s: tcp=(%.4f, %.4f, %.4f) "
              "object=(%.4f, %.4f, %.4f) delta=(%.4f, %.4f, %.4f) "
              "center_dist=%.4f m <= %.4f m, surface_dist=%.4f m",
              label.c_str(), object_name.c_str(), tcp.x, tcp.y, tcp.z, object_position.x,
              object_position.y, object_position.z, dx, dy, dz, distance.center, max_distance,
              distance.surface);
  return true;
}

/**
 * cleanupObject() — 抓取失败时的清理操作
 *
 *  抓取流程失败时张开夹爪，并幂等调用 detach 清理可能存在的固定关节。
 */
void cleanupObject(PickNode& node, const std::string& object_name) {
  RCLCPP_WARN(node.get_logger(), "Cleaning up %s: open gripper and detach if needed.",
              object_name.c_str());
  node.openGripper();
  node.detachObject(object_name);
}

/**
 * pickObject() — 抓取单个物体的开放夹爪 attach 转移流程
 *
 *  ┌──────┬────────────────┬─────────────────────────────────────┐
 *  │ 步数 │ 动作           │ 描述                                │
 *  ├──────┼────────────────┼─────────────────────────────────────┤
 *  │  1   │ approach       │ 从当前位置移动到物体正上方安全高度    │
 *  │  2   │ descent        │ 直线下降至抓取高度（接触物体）       │
 *  │  3   │ attach+lift    │ 夹爪保持张开，attach 后垂直抬高 10cm │
 *  │  4   │ transfer       │ 夹爪保持张开，移动到放置点           │
 *  │  5   │ detach         │ 在放置点 detach                      │
 *  └──────┴────────────────┴─────────────────────────────────────┘
 *
 *  全程不执行 closeGripper()，依靠 Gazebo fixed joint 模拟吸附搬运。
 *
 *  @return true=该物体抓取成功
 */
bool pickObject(PickNode& node, moveit::planning_interface::MoveGroupInterface& move_group,
                Object object, double approach_clearance, double lift_z, double grasp_z_offset,
                double min_grasp_z, double release_z, double tool_z_offset,
                double max_grasp_distance,
                double open_gripper_lift_height, double back_place_x, double back_place_y,
                const geometry_msgs::msg::Quaternion& back_place_orientation, bool is_last) {
  // ── 步骤 0：读取物体实时位置 ──
  geometry_msgs::msg::Quaternion orientation;
  // GraspNet 6D 路径专用的接近间隙（与位置回退的 approach_clearance 相互独立）
  const double graspnet_approach_clearance =
      node.get_parameter_or<double>("graspnet_6d_approach_clearance", 0.0);
  bool using_position_only_fallback = false;
  // 候选预检前同步物体最新位姿到规划场景（物体可能被碰撞推移/倾斜）
  if (node.moveitSceneEnabled()) {
    node.refreshObjectCollisionPose(object.name);
  }

  if (!chooseReachableGraspCandidate(node, move_group, object, graspnet_approach_clearance,
                                     lift_z, grasp_z_offset, min_grasp_z, tool_z_offset,
                                     open_gripper_lift_height, back_place_x, back_place_y,
                                     back_place_orientation, object, orientation)) {
    const bool enable_position_fallback =
        node.get_parameter_or<bool>("enable_position_only_fallback", true);
    if (!enable_position_fallback) {
      RCLCPP_ERROR(node.get_logger(),
                   "%s has no reachable GraspNet 6D pose and "
                   "enable_position_only_fallback=false.",
                   object.name.c_str());
      return false;
    }
    // 6D 候选全部不可达 → 位置回退：忽略 GraspNet 方向预测，
    // 用俯视姿态（topDownTcpOrientation）下压抓取（可经参数禁用）
    RCLCPP_WARN(node.get_logger(),
                "%s has no reachable GraspNet 6D pose in the limited planner pass; "
                "trying the isolated position-only baseline.",
                object.name.c_str());
    if (!chooseReachablePositionGraspCandidate(
            node, move_group, object, approach_clearance, lift_z, grasp_z_offset, min_grasp_z,
            tool_z_offset, open_gripper_lift_height, back_place_x, back_place_y,
            back_place_orientation, object, orientation)) {
      return false;
    }
    using_position_only_fallback = true;
  }

  // 预先张开夹爪：本次抓取全程不闭爪，靠 Gazebo 固定关节吸附搬运
  node.openGripper();

  // 计算偏移和完整位姿序列
  (void)release_z;
  (void)is_last;
  const double execution_approach_clearance =
      using_position_only_fallback ? approach_clearance : graspnet_approach_clearance;
  const auto poses = makeObjectPoses(object, orientation, execution_approach_clearance, lift_z,
                                     grasp_z_offset, min_grasp_z, release_z, tool_z_offset, false);
  const double grasp_z = poses.grasp.position.z; // 抓取 TCP 高度（日志用）
  // open_lift_pose：张开夹爪垂直抬升 open_gripper_lift_height 的目标位姿
  auto open_lift_pose = poses.grasp;
  open_lift_pose.position.z += open_gripper_lift_height;
  // back_place_pose：带物平移到放置点，姿态沿用 home 姿态（放置朝向一致）
  auto back_place_pose = open_lift_pose;
  back_place_pose.position.x = back_place_x;
  back_place_pose.position.y = back_place_y;
  back_place_pose.orientation = back_place_orientation;

  RCLCPP_INFO(node.get_logger(),
              "Picking %s (%s) from camera 6D grasp target=(%.4f, %.4f, %.4f), "
              "q=(%.4f, %.4f, %.4f, %.4f), grasp_tcp_z=%.4f, virtual TCP offset=%.4f",
              object.name.c_str(), object.shape.c_str(), object.x, object.y, object.z,
              orientation.x, orientation.y, orientation.z, orientation.w, grasp_z, tool_z_offset);

  // ── 执行前预检的最后 2 段（approach/grasp 已在候选遍历中预检过）──
  // 1) 张开夹爪抬升位姿必须可达：attach 后要带着物体抬升，不可达则拒绝
  if (!canPlanFullPose(node, move_group, open_lift_pose,
                       object.name + " preflight open-gripper lift")) {
    RCLCPP_ERROR(node.get_logger(),
                 "%s rejected before attach: open-gripper lift pose is not reachable.",
                 object.name.c_str());
    return false;
  }
  // 2) 放置位姿必须可达：以选中姿态带物平移放置，不可达则拒绝。
  //    两段预检均发生在 attach 之前——此时拒绝只需张开夹爪，
  //    attach 之后失败则要清理 ACO，代价高得多
  if (!canPlanFullPose(node, move_group, back_place_pose,
                       object.name + " preflight place target")) {
    RCLCPP_ERROR(node.get_logger(),
                 "%s rejected before attach: place target TCP=(%.3f, %.3f, %.3f) "
                 "is not reachable with the selected GraspNet orientation.",
                 object.name.c_str(), back_place_pose.position.x, back_place_pose.position.y,
                 back_place_pose.position.z);
    return false;
  }

  // ── 步骤 1: 到达 GraspNet 原始抓取位姿 ──
  RCLCPP_INFO(node.get_logger(), "%s step 1/8: move to exact GraspNet TCP=(%.3f, %.3f, %.3f)",
              object.name.c_str(), poses.approach.position.x, poses.approach.position.y,
              poses.approach.position.z);
  if (!moveLinear(node, move_group, poses.approach, object.name + " approach")) {
    RCLCPP_ERROR(node.get_logger(), "%s FAILED at approach", object.name.c_str());
    return false;
  }

  // ── 步骤 2: descent（下降 — 直线下降到抓取高度） ──
  RCLCPP_INFO(node.get_logger(), "%s step 2/8: descent to exact GraspNet TCP=(%.3f, %.3f, %.3f)",
              object.name.c_str(), poses.grasp.position.x, poses.grasp.position.y,
              poses.grasp.position.z);
  if (!moveLinear(node, move_group, poses.grasp, object.name + " descent")) {
    RCLCPP_ERROR(node.get_logger(), "%s FAILED at descent", object.name.c_str());
    return false;
  }

  // 门控检查链第 1 道：抓取点处 TCP 必须贴近物体（≤max_grasp_distance，默认 0.08m）
  if (!verifyCurrentTcpNearObject(node, move_group, object.name, tool_z_offset,
                                  max_grasp_distance, "pre-attach open-gripper grasp")) {
    node.openGripper();
    RCLCPP_ERROR(node.get_logger(), "%s FAILED before attach: TCP is too far.",
                 object.name.c_str());
    return false;
  }

  // 门控检查链第 2 道（attach 内部）：TCP 中心与物体中心
  // ≤ kMaxAttachCenterDistance，且要求夹爪张开 —— 拒绝"闭爪硬怼"式抓取
  if (!node.attachObjectAfterOpenGripperDistanceGate(move_group, object.name)) {
    node.openGripper();
    RCLCPP_ERROR(
        node.get_logger(),
        "%s FAILED before open-gripper transfer: attach gate rejected TCP center distance.",
        object.name.c_str());
    return false;
  }

  // ── 步骤 3: 保持夹爪张开，attach 后垂直抬高指定高度 ──
  RCLCPP_INFO(node.get_logger(),
              "%s step 3/5: keep gripper open, lift attached object by %.3f m -> "
              "TCP=(%.3f, %.3f, %.3f)",
              object.name.c_str(), open_gripper_lift_height, open_lift_pose.position.x,
              open_lift_pose.position.y, open_lift_pose.position.z);
  if (!moveLinear(node, move_group, open_lift_pose, object.name + " open-gripper lift")) {
    cleanupObject(node, object.name);
    RCLCPP_ERROR(node.get_logger(), "%s FAILED at open-gripper lift", object.name.c_str());
    return false;
  }
  // 门控检查链第 3 道：抬升后物体必须跟随 TCP（attach 未生效时此门控拦截）
  if (!verifyCurrentTcpNearObject(node, move_group, object.name, tool_z_offset,
                                  max_grasp_distance, "post-open-gripper-lift grasp")) {
    cleanupObject(node, object.name);
    RCLCPP_ERROR(node.get_logger(),
                 "%s FAILED after open-gripper lift: attach did not carry the object.",
                 object.name.c_str());
    return false;
  }

  // ── 步骤 4: 保持夹爪张开，移动到放置目标点 ──
  RCLCPP_INFO(node.get_logger(),
              "%s step 4/5: keep gripper open, move to place target -> "
              "TCP=(%.3f, %.3f, %.3f)",
              object.name.c_str(), back_place_pose.position.x, back_place_pose.position.y,
              back_place_pose.position.z);
  if (!moveLinear(node, move_group, back_place_pose, object.name + " place transfer")) {
    // 附着物体可能使搬运段碰撞检查失败：摘除 ACO（只摘不挂）后重试一次。
    // 摘除后 transfer 仍无避碰对象，可回到改动前行为完成搬运。
    // 这是"预检通过但执行仍失败"场景的兜底：预检时场景中没有附体
    // ACO，执行时带附体物体碰撞约束更严，失败后摘除 ACO 重试可继续完成
    bool transferred = false;
    if (node.moveitSceneEnabled() &&
        node.get_parameter_or<bool>("moveit_scene_transfer_fallback", true)) {
      RCLCPP_WARN(node.get_logger(),
                  "%s place transfer failed with attached object; "
                  "retrying with object removed from scene.",
                  object.name.c_str());
      node.detachObjectMoveIt(object.name, nullptr, false);
      transferred =
          moveLinear(node, move_group, back_place_pose, object.name + " place transfer (no scene)");
    }
    if (!transferred) {
      cleanupObject(node, object.name);
      RCLCPP_ERROR(node.get_logger(), "%s FAILED at place transfer", object.name.c_str());
      return false;
    }
  }
  // 门控检查链第 4 道：转移后物体仍在 TCP 附近（防中途脱落未被发现）
  if (!verifyCurrentTcpNearObject(node, move_group, object.name, tool_z_offset,
                                  max_grasp_distance, "post-place-transfer grasp")) {
    cleanupObject(node, object.name);
    RCLCPP_ERROR(node.get_logger(), "%s FAILED after place transfer: object drifted away from TCP.",
                 object.name.c_str());
    return false;
  }

  // ── 步骤 5: 仍保持夹爪张开，在放置目标点 detach ──
  RCLCPP_INFO(node.get_logger(), "%s step 5/5: detach with gripper still open",
              object.name.c_str());
  if (!node.detachObject(object.name)) {
    RCLCPP_ERROR(node.get_logger(), "%s FAILED at detach", object.name.c_str());
    return false;
  }
  // detach 后等待 0.5s：让物体在 Gazebo 中落地稳定，再做位置校验
  rclcpp::sleep_for(500ms);
  {
    // 期望释放位置 = 放置点：校验物体确实落在放置点（容差 0.08m，仅告警）
    geometry_msgs::msg::Point expected_release;
    expected_release.x = back_place_pose.position.x;
    expected_release.y = back_place_pose.position.y;
    expected_release.z = back_place_pose.position.z;
    node.verifyObjectNear(object.name, expected_release, 0.08, "post-open-gripper place detach");
  }

  return true;
}

} // namespace piper_gazebo::pick_place

using namespace piper_gazebo::pick_place;

// ============================================================================
//  main() — 程序入口
// ============================================================================

/**
 * main() — 抓取程序主入口
 *
 *  【启动流程】
 *  1. rclcpp::init — 初始化 ROS2
 *  2. 创建 PickNode 实例
 *  3. 后台线程执行 executor.spin() 处理 ROS2 回调
 *  4. 等待所有运行时依赖就绪（8 秒超时）
 *  5. 等待 MoveIt2 move_group Action 就绪（8 秒超时）
 *  6. 创建 MoveGroupInterface，配置规划参数
 *  7. 从参数服务器读取可调参数
 *  8. 回到 home 姿态（重试最多 5 次）
 *  9. 主循环：
 *     a. 最多 max_place_attempts 轮（默认 3 轮）
 *     b. 每轮遍历未完成的所有物体，尝试 pickObject()
 *     c. 成功一个就回 home 再抓下一个
 *  10. 回到零位，关机
 *
 *  @return 0=全部成功, 1=部分失败, 2=运行时依赖缺失, 3=home 运动失败
 */
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PickNode>();

  // 复位感知状态文件（覆盖上次运行残留），先于感知等待：
  // Docker 侧据此只识别尚未放置的物体。
  node->writePickStateFile({});

  // ── 后台 ROS2 事件循环 ──
  // 在独立线程中 spin，使 Service/Action 回调可以正常触发
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  auto shutdown = [&]() {
    executor.cancel();
    if (spinner.joinable()) {
      spinner.join();
    }
    rclcpp::shutdown();
  };

  // ── 等待依赖就绪 ──
  if (!node->waitForRuntimeDependencies(30s) || !waitForMoveGroupAction(node)) {
    shutdown();
    return 2; // 启动失败：依赖缺失
  }

  // ── 初始化 MoveIt MoveGroup ──
  moveit::planning_interface::MoveGroupInterface move_group(node, kMoveGroupName);
  configureMoveGroup(move_group);

  // ── 物体加入 MoveIt 规划场景 + ACM 豁免（RViz 可见 + OMPL 避碰） ──
  // 任一步失败仅降级：继续无场景运行，不回归原有行为。
  if (node->moveitSceneEnabled()) {
    if (!node->initPlanningSceneInterface()) {
      RCLCPP_ERROR(node->get_logger(),
                   "PlanningSceneInterface unavailable; continuing WITHOUT scene objects.");
    } else {
      if (node->addObjectCollisionObjects()) {
        RCLCPP_INFO(node->get_logger(), "3 target objects added to MoveIt planning scene.");
      } else {
        RCLCPP_ERROR(node->get_logger(),
                     "Failed to add objects to planning scene; continuing degraded.");
      }
      if (node->applyGripperObjectAcm(move_group)) {
        RCLCPP_INFO(node->get_logger(), "Gripper-object ACM exemptions applied.");
      }
    }
  }

  // ── 从参数服务器读取可调参数 ──
  // 这些参数可以通过 launch 文件或 YAML 动态调整
  const double approach_clearance = node->get_parameter_or<double>("approach_clearance", 0.18);
  const double lift_z = node->get_parameter_or<double>("lift_z", 0.24);
  const double grasp_z_offset = node->get_parameter_or<double>("grasp_z_offset", 0.0);
  const double min_grasp_z = node->get_parameter_or<double>("min_grasp_z", 0.0);
  const double release_z = node->get_parameter_or<double>("release_z", 0.150);
  const double tool_z_offset = node->get_parameter_or<double>("tool_z_offset", 0.0);
  const double max_grasp_distance =
      node->get_parameter_or<double>("max_grasp_distance", kDefaultMaxGraspDistance);
  const double open_gripper_lift_height =
      node->get_parameter_or<double>("open_gripper_lift_height", 0.07);
  const double back_place_x = node->get_parameter_or<double>("back_place_x", 0.30);
  const double back_place_y = node->get_parameter_or<double>("back_place_y", -0.35);
  // max_place_attempts removed: loop now runs until all objects are placed

  // Unpause GraspNet after placing so it produces fresh candidates for
  // the next object. The node then re-pauses before moving the arm.
  if (!node->waitForFreshPerceptionTargets(120s)) {
    shutdown();
    return 2;
  }

  // ── home 关节位置（弧度） ──
  // 该位置是机械臂的初始/预备姿态，使得后续 Cartesian 路径有足够的
  // 运动空间，避免关节限位或奇异点
  // 根据目标物体位置调整 joint1，使 IK 种子起始位姿更接近可达解
  // per-object home 表：物体在 y 方向分布（red_box 偏左 / blue_sphere 偏右），
  // 仅 joint1 随物体方位微调 —— 让规划起点朝向目标物体侧，ssik 解析解
  // 与 OMPL 起始树更接近解域；其余 5 个关节固定，保证姿态一致便于复用
  auto home_for_object = [](const std::string& object_name) {
    // green_cylinder (y=0.02, 正前方) → joint1=0
    // red_box (y=-0.16, 左侧) → joint1=-0.35
    // blue_sphere (y=0.17, 右侧) → joint1=0.35
    if (object_name == "red_box") {
      return std::vector<double>{-0.35, 0.55, -0.75, 0.0, 0.55, 0.0};
    }
    if (object_name == "blue_sphere") {
      return std::vector<double>{0.35, 0.55, -0.75, 0.0, 0.55, 0.0};
    }
    return std::vector<double>{0.0, 0.55, -0.75, 0.0, 0.55, 0.0};
  };

  // ── 回到 home ──
  // MoveIt 的 ActionClient 首次连接 arm_controller 可能需要时间，
  // 因此重试最多 5 次，每次间隔 2 秒
  bool home_reached = false;
  for (int retry = 0; retry < 5 && !home_reached; ++retry) {
    if (retry > 0) {
      RCLCPP_INFO(node->get_logger(), "Retrying home move (attempt %d/5)...", retry + 1);
      rclcpp::sleep_for(2s);
    }
    home_reached = moveJoints(*node, move_group, home_for_object("green_cylinder"), "home");
  }
  if (!home_reached) {
    RCLCPP_ERROR(node->get_logger(), "Failed to reach home position after 5 attempts.");
    shutdown();
    return 3; // 启动失败：home 运动失败
  }
  rclcpp::sleep_for(300ms);
  // 以 home 姿态的四元数作为统一放置朝向（所有物体的放置姿态一致）
  const auto back_place_orientation = move_group.getCurrentPose(kEndEffectorLink).pose.orientation;
  RCLCPP_INFO(node->get_logger(),
              "Using home TCP orientation for placement: "
              "q=(%.4f, %.4f, %.4f, %.4f).",
              back_place_orientation.x, back_place_orientation.y, back_place_orientation.z,
              back_place_orientation.w);

  // ── 抓取主循环 ──
  node->openGripper();

  // 待抓取列表：从定义顺序出发，默认随机打乱（每个物体的 home/感知均按名称独立处理）
  auto item_list = objects();
  // 抓取顺序随机化：默认开启。objects() 中的顺序仅为定义顺序，
  // 不承载功能依赖（各物体 home 姿态、感知刷新均按名称独立处理）。
  const bool randomize_pick_order = node->get_parameter_or<bool>("randomize_pick_order", true);
  if (randomize_pick_order) {
    std::shuffle(item_list.begin(), item_list.end(), std::mt19937(std::random_device{}()));
    std::string order;
    for (const auto& obj : item_list) {
      if (!order.empty())
        order += " -> ";
      order += obj.name;
    }
    RCLCPP_INFO(node->get_logger(), "Randomized pick order: %s", order.c_str());
  }
  bool arm_at_zero = false;
  std::set<std::string> placed_objects;
  int attempt = 0;
  // 位置回退开关：6D 候选全不可达时是否降级到仅位置抓取（默认开启；
  // 关闭时任一物体失败将触发 abort 主循环）
  const bool enable_position_fallback =
      node->get_parameter_or<bool>("enable_position_only_fallback", true);
  const bool graspnet_scan_only = node->get_parameter_or<bool>("graspnet_6d_scan_only", false);
  const std::string graspnet_scan_object =
      node->get_parameter_or<std::string>("graspnet_6d_scan_object", "");
  // 失败即中止策略标志：置位后主循环立即退出（见下方 enable_position_fallback 分支）
  bool abort_pick_loop = false;

  // 主循环：直到全部物体放置完成，或 abort_pick_loop 置位（失败即中止）
  while (placed_objects.size() < item_list.size() && !abort_pick_loop) {
    ++attempt;
    RCLCPP_INFO(node->get_logger(), "Pick attempt %d: %zu/%zu objects already placed.", attempt,
                placed_objects.size(), item_list.size());

    for (const auto& object : item_list) {
      // 已放置的物体直接跳过（每轮只处理未完成物体）
      if (placed_objects.count(object.name) > 0) {
        continue;
      }
      if (graspnet_scan_only && !graspnet_scan_object.empty() &&
          object.name != graspnet_scan_object) {
        RCLCPP_INFO(node->get_logger(), "Scan-only mode skipping %s; target object is %s.",
                    object.name.c_str(), graspnet_scan_object.c_str());
        placed_objects.insert(object.name);
        node->writePickStateFile(placed_objects);
        continue;
      }

      // 本次抓取动作会改变臂位，标记"不在零位"；最终归零以实际臂位为准
      arm_at_zero = false;

      // Delay between objects to let Docker produce fresh targets
      if (placed_objects.size() + (attempt > 1 ? 1 : 0) > 0) {
        RCLCPP_INFO(node->get_logger(), "Waiting for fresh GraspNet inference before %s...",
                    object.name.c_str());
        rclcpp::sleep_for(3s);
      }

      // 每个物体抓取前先回到该物体的规划 home（最多重试 3 次）：
      // 保证 ssik 解析 IK 种子与 OMPL 起始树落在这个物体可达域内
      bool object_home_ok = false;
      for (int r = 0; r < 3; ++r) {
        if (r > 0) {
          rclcpp::sleep_for(1s);
        }
        if (moveJoints(*node, move_group, home_for_object(object.name),
                       object.name + " planning home")) {
          object_home_ok = true;
          break;
        }
      }
      if (!object_home_ok) {
        RCLCPP_ERROR(node->get_logger(), "Failed to reach planning home for %s; retry later.",
                     object.name.c_str());
        rclcpp::sleep_for(5s);
        continue;
      }
      arm_at_zero = false;

      const bool picked =
          pickObject(*node, move_group, object, approach_clearance, lift_z, grasp_z_offset,
                     min_grasp_z, release_z, tool_z_offset, max_grasp_distance,
                     open_gripper_lift_height, back_place_x, back_place_y,
                     back_place_orientation, object.name == item_list.back().name);
      arm_at_zero = false;

      if (picked) {
        placed_objects.insert(object.name);
        // 感知状态更新：该物体从 pending 移除，Docker 不再识别它
        node->writePickStateFile(placed_objects);
        RCLCPP_INFO(node->get_logger(), "%s released at drop pose (%zu/%zu).", object.name.c_str(),
                    placed_objects.size(), item_list.size());
      } else if (graspnet_scan_only) {
        RCLCPP_INFO(node->get_logger(),
                    "Scan-only mode finished for %s; continuing to next object.",
                    object.name.c_str());
      } else if (!enable_position_fallback) {
        // 失败即中止主循环（abort）：该物体无任何可达候选、且位置回退被
        // 禁用。相机/外参/工作区条件相同的情况下，其余物体大概率同样
        // 失败（历史日志显示 red_box 移至工作区边缘后 ssik 0 解、每次白耗
        // 数分钟预检），继续循环无意义 —— 立即退出并返回部分成功状态
        RCLCPP_ERROR(node->get_logger(),
                     "Stopping pick loop because %s has no reachable GraspNet 6D "
                     "plan and position-only fallback is disabled.",
                     object.name.c_str());
        abort_pick_loop = true;
        break;
      }

      if (!graspnet_scan_only && placed_objects.size() < item_list.size()) {
        std::set<std::string> remaining_objects;
        for (const auto& candidate : item_list) {
          if (placed_objects.count(candidate.name) == 0) {
            remaining_objects.insert(candidate.name);
          }
        }
        if (node->tryRefreshPerceptionCache(remaining_objects)) {
          RCLCPP_INFO(node->get_logger(), "Fresh GraspNet CSV loaded for remaining object(s).");
        }
        rclcpp::sleep_for(200ms);
      }
    }
    if (graspnet_scan_only) {
      RCLCPP_INFO(node->get_logger(), "Scan-only mode completed one pass over all objects.");
      break;
    }
  }
  // ── 回到最终零位 ──
  // arm_at_zero 为 true 说明臂从未离开零位（如 scan-only 提前退出），无需再运动
  const std::vector<double> final_joints{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  const bool final_home_reached =
      arm_at_zero || moveJoints(*node, move_group, final_joints, "final zero");
  if (!final_home_reached) {
    RCLCPP_ERROR(node->get_logger(), "Final zero move failed.");
  }

  RCLCPP_INFO(node->get_logger(), "Placed %zu/%zu required objects.", placed_objects.size(),
              item_list.size());

  // 返回码：0=全部放置成功, 1=部分失败（抓取失败或 abort 中止）
  const bool all_objects_placed = placed_objects.size() == item_list.size();
  shutdown();
  return all_objects_placed ? 0 : 1;
}

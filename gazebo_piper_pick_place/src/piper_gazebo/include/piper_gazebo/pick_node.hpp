#pragma once

#include <chrono>
#include <ctime>
#include <fstream>
#include <future>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <sys/stat.h>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <gazebo_msgs/srv/get_entity_state.hpp>
// moveit 的 urdf/urdf/model.h 需要完整类型定义（必须位于 moveit include 之前）
#include <urdfdom_headers/urdf_model/model.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/allowed_collision_matrix.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <srdfdom/model.h>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include "piper_gazebo/grasp_common.hpp"
#include "piper_gazebo/srv/attach_detach.hpp"

namespace piper_gazebo::pick_place {
using namespace std::chrono_literals;

/**
 * PickNode — 抓取节点核心类（一个 ROS2 节点，承担全部感知/场景/执行接口）
 *
 *  职责分组：
 *    - 感知读取：readObjectPosition/readObjectPose（Gazebo 实时状态）+ 感知缓存
 *      读取与视觉↔Gazebo 交叉校验（readPerceptionObjectPosition /
 *      readValidatedVisionObject*）
 *    - 场景管理：MoveIt 规划场景物体同步（addObjectCollisionObjects /
 *      refreshObjectCollisionPose / attachObjectMoveIt / detachObjectMoveIt /
 *      applyGripperObjectAcm）
 *    - attach/detach：Gazebo 固定关节吸附（/piper/attach、/piper/detach）
 *      与 MoveIt ACO 同步的组合操作（attachObjectAfterOpenGripperDistanceGate）
 *    - 状态写入：writePickStateFile —— pick_state.json 告知 Docker 感知
 *      哪些物体仍待抓取（pending）/已放置（placed）
 *    - 夹爪控制：openGripper/closeGripper（joint7/joint8 对称运动）
 *
 *  Gazebo 的固定关节 attach 是"吸附"事实，MoveIt ACO 只是规划认知 ——
 *  因此所有 attach/detach 同步失败均降级告警、不阻断流程
 */
class PickNode : public rclcpp::Node
{
public:
  // MoveIt 规划场景集成常量
  static constexpr const char* kAttachLink = "link6"; // 与 Gazebo attach robot_link 一致
  static constexpr const char* kGripperTouchLinks[] = {"link7", "link8", "gripper_base",
                                                       "grasp_tcp"};

  // 构造函数：创建夹爪 Action Client + Gazebo/attach/detach Service Client。
  // 参数经 automatically_declare_parameters_from_overrides 从 launch 注入
  PickNode()
      : Node(kNodeName,
             rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) {
    // 夹爪控制：通过 FollowJointTrajectory Action 发送位置命令。
    // 该 Action 接口由 Gazebo 中的 gripper_controller
    // （JointTrajectoryController）提供，与 MoveIt 使用同一接口。
    gripper_action_client_ =
        rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
            this, "/gripper_controller/follow_joint_trajectory");

    // Gazebo 物体状态查询服务
    get_entity_client_ = create_client<gazebo_msgs::srv::GetEntityState>("/get_entity_state");
    attach_client_ = create_client<piper_gazebo::srv::AttachDetach>("/piper/attach");
    detach_client_ = create_client<piper_gazebo::srv::AttachDetach>("/piper/detach");
  }

  /**
   * waitForRuntimeDependencies() — 等待所有运行时依赖就绪
   *
   *  在倒计时 timeout 内，以 5Hz 轮询检测以下依赖是否全部可用：
   *    - Gazebo 夹爪 Action Server
   *    - /get_entity_state Service
   *
   *  超时后调用 reportMissingRuntimeDependency() 输出诊断日志。
   *
   *  @param timeout  超时时长
   *  @return true=全部就绪, false=超时
   */
  bool waitForRuntimeDependencies(const std::chrono::seconds timeout) {
    const auto deadline = now() + rclcpp::Duration(timeout);
    rclcpp::Rate rate(5);
    bool wait_reported = false;

    while (rclcpp::ok() && now() < deadline) {
      const bool gripper_ready = gripper_action_client_->wait_for_action_server(1ms);
      const bool gazebo_state_ready = get_entity_client_->service_is_ready();
      const bool attach_ready = attach_client_->service_is_ready();
      const bool detach_ready = detach_client_->service_is_ready();

      if (gripper_ready && gazebo_state_ready && attach_ready && detach_ready) {
        return true;
      }

      if (!wait_reported) {
        RCLCPP_INFO(get_logger(), "Waiting for Gazebo state service and gripper controller.");
        wait_reported = true;
      }
      rate.sleep();
    }

    reportMissingRuntimeDependency();
    return false;
  }

  /**
   * openGripper() — 张开夹爪
   *
   *  发送 joint7=0.035, joint8=-0.035 使夹爪两侧张开。
   *  正值表示张开宽度（米），两个关节对称运动。
   */
  void openGripper() {
    commandGripper(0.035, -0.035, 0.8);
    rclcpp::sleep_for(500ms);
  }

  /**
   * closeGripper() — 闭合夹爪
   *
   *  真实夹爪闭合发生在 attach 门控通过之后；attach 门控要求夹爪已张开且
   *  TCP 中心到物体中心距离小于 4cm。
   */
  void closeGripper() {
    commandGripper(0.0, 0.0, 2.0);
    rclcpp::sleep_for(800ms);
  }

  /**
   * attachObjectAfterOpenGripperDistanceGate() — 距离门控 + attach 组合操作
   *
   *  前置条件：夹爪张开、TCP 已到达抓取点。流程：
   *    1. 读取物体与 TCP 位置，中心距离门控（kMaxAttachCenterDistance=0.08m）
   *    2. 通过后调用 Gazebo /piper/attach 建立固定关节（吸附）
   *    3. 再同步 MoveIt ACO（失败仅降级：Gazebo 物理吸附才是事实）
   *
   *  @return true=attach 成功（Gazebo 侧）
   */
  bool attachObjectAfterOpenGripperDistanceGate(
      moveit::planning_interface::MoveGroupInterface& move_group, const std::string& object_name) {
    geometry_msgs::msg::Pose object_pose;
    if (!readObjectPose(object_name, object_pose)) {
      RCLCPP_ERROR(get_logger(), "Attach gate failed for %s: cannot read Gazebo object pose.",
                   object_name.c_str());
      return false;
    }
    const auto object_position = object_pose.position;

    const auto tcp = move_group.getCurrentPose(kEndEffectorLink).pose.position;
    const auto distance = distanceToVerticalCylinder(tcp, object_position);
    if (distance.center > kMaxAttachCenterDistance) {
      RCLCPP_ERROR(get_logger(),
                   "Attach gate FAIL for %s: gripper must be open and TCP center must be "
                   "within %.4f m of the object center before attach. "
                   "tcp=(%.4f, %.4f, %.4f), object=(%.4f, %.4f, %.4f), "
                   "surface_dist=%.4f m, center_dist=%.4f m. Refusing attach.",
                   object_name.c_str(), kMaxAttachCenterDistance, tcp.x, tcp.y, tcp.z,
                   object_position.x, object_position.y, object_position.z, distance.surface,
                   distance.center);
      return false;
    }

    RCLCPP_INFO(get_logger(),
                "Attach gate PASS for %s with gripper open: center_dist=%.4f m <= %.4f m, "
                "surface_dist=%.4f m.",
                object_name.c_str(), distance.center, kMaxAttachCenterDistance, distance.surface);
    if (!callAttachDetach(attach_client_, "attach", object_name)) {
      return false;
    }
    // Gazebo 固定关节 attach 成功后，同步 MoveIt 认知（物体随 link6 移动）。
    // 失败仅降级不阻断：Gazebo 物理吸附是事实，MoveIt attach 只是规划认知。
    if (!attachObjectMoveIt(object_name, &object_pose)) {
      RCLCPP_WARN(get_logger(),
                  "MoveIt attach failed for %s (Gazebo attach OK); continuing degraded.",
                  object_name.c_str());
    }
    return true;
  }

  // detach：先解除 Gazebo 固定关节，再移除 MoveIt ACO（失败仅降级）
  bool detachObject(const std::string& object_name) {
    if (!callAttachDetach(detach_client_, "detach", object_name)) {
      return false;
    }
    if (!detachObjectMoveIt(object_name)) {
      RCLCPP_WARN(get_logger(), "MoveIt detach failed for %s (Gazebo detach OK).",
                  object_name.c_str());
    }
    return true;
  }

  // ===================================================================
  //  MoveIt 规划场景集成 — 物体可见 + OMPL 避碰 + attach 同步
  // ===================================================================

  /**
   * writePickStateFile() — 原子写感知状态文件（shared_dir/pick_state.json）。
   * 告知 Docker 推理侧哪些物体仍待抓取（pending）/已放置（placed），
   * 使感知只识别未抓取物体。实现见 src/pick_state_writer.cpp。
   */
  bool writePickStateFile(const std::set<std::string>& placed_objects);

  // 是否启用 MoveIt 规划场景集成（launch 参数 enable_moveit_scene_objects）
  bool moveitSceneEnabled() const {
    return get_parameter_or<bool>("enable_moveit_scene_objects", true);
  }

  bool initPlanningSceneInterface() {
    if (!planning_scene_interface_) {
      planning_scene_interface_ =
          std::make_unique<moveit::planning_interface::PlanningSceneInterface>();
    }
    return planning_scene_interface_ != nullptr;
  }

  // ===================================================================
  //  感知读取 — Gazebo 实时状态 + 视觉缓存（含交叉校验门控）
  // ===================================================================

  /**
   * readObjectPose() — 读取物体完整位姿（position + orientation，世界系）
   * 与 readObjectPosition 相同骨架，差异是整体赋值 response->state.pose。
   */
  bool readObjectPose(const std::string& name, geometry_msgs::msg::Pose& pose) {
    if (!get_entity_client_->wait_for_service(5s)) {
      RCLCPP_ERROR(get_logger(), "Gazebo /get_entity_state service is not available.");
      return false;
    }

    auto request = std::make_shared<gazebo_msgs::srv::GetEntityState::Request>();
    request->name = name;
    request->reference_frame = "world";

    auto future = get_entity_client_->async_send_request(request);
    if (future.wait_for(3s) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "Timed out reading Gazebo entity %s.", name.c_str());
      return false;
    }

    const auto response = future.get();
    if (!response->success) {
      RCLCPP_ERROR(get_logger(), "Gazebo could not read state for %s.", name.c_str());
      return false;
    }

    pose = response->state.pose;
    return true;
  }

  /** 把物体封装成 MoveIt 碰撞对象（竖直圆柱：HEIGHT=0.0675m, RADIUS=0.024m） */
  void makeObjectCollisionObject(const std::string& name, const geometry_msgs::msg::Pose& pose,
                                 moveit_msgs::msg::CollisionObject& co,
                                 uint8_t operation = moveit_msgs::msg::CollisionObject::ADD) {
    co.header.frame_id = "world";
    co.header.stamp = this->now();
    co.id = name;
    co.operation = operation;
    // Humble 的 CYLINDER 维度顺序：dimensions[0]=HEIGHT, dimensions[1]=RADIUS
    shape_msgs::msg::SolidPrimitive prim;
    prim.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    prim.dimensions.resize(2);
    prim.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] = kObjectCylinderLength;
    prim.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS] = kObjectCylinderRadius;
    co.primitives.push_back(prim);
    co.primitive_poses.push_back(pose);
  }

  /** 启动时把 3 个物体按 Gazebo 实时位姿加入规划场景。 */
  bool addObjectCollisionObjects() {
    if (!moveitSceneEnabled() || !initPlanningSceneInterface()) {
      return false;
    }
    std::vector<moveit_msgs::msg::CollisionObject> object_msgs;
    bool all_ok = true;
    for (const auto& obj : objects()) {
      geometry_msgs::msg::Pose pose;
      if (!readObjectPose(obj.name, pose)) {
        all_ok = false;
        continue;
      }
      moveit_msgs::msg::CollisionObject co;
      makeObjectCollisionObject(obj.name, pose, co);
      object_msgs.push_back(co);
    }
    if (!object_msgs.empty()) {
      all_ok = planning_scene_interface_->applyCollisionObjects(object_msgs) && all_ok;
    }
    return all_ok;
  }

  /** 抓取前用 ADD（幂等替换，规避 MOVE 必须空几何）同步物体最新位姿。 */
  bool refreshObjectCollisionPose(const std::string& name) {
    if (!moveitSceneEnabled() || !initPlanningSceneInterface()) {
      return false;
    }
    geometry_msgs::msg::Pose pose;
    if (!readObjectPose(name, pose)) {
      return false;
    }
    moveit_msgs::msg::CollisionObject co;
    makeObjectCollisionObject(name, pose, co);
    return planning_scene_interface_->applyCollisionObject(co);
  }

  /**
   * attachObjectMoveIt() — Gazebo attach 成功后同步 MoveIt 认知。
   * ACO 挂在 link6（与 Gazebo 固定关节父 link 一致），pose 按 world 帧解释，
   * move_group 自动换算为 link6 相对位姿；touch_links = 与物体接触的夹爪 link。
   */
  bool attachObjectMoveIt(const std::string& name, const geometry_msgs::msg::Pose* pose = nullptr) {
    if (!moveitSceneEnabled() || !initPlanningSceneInterface()) {
      return false;
    }
    geometry_msgs::msg::Pose p;
    if (pose) {
      p = *pose;
    } else if (!readObjectPose(name, p)) {
      return false;
    }

    moveit_msgs::msg::AttachedCollisionObject aco;
    aco.link_name = kAttachLink;
    makeObjectCollisionObject(name, p, aco.object);
    aco.touch_links = {kGripperTouchLinks[0], kGripperTouchLinks[1], kGripperTouchLinks[2],
                       kGripperTouchLinks[3]};
    const bool ok = planning_scene_interface_->applyAttachedCollisionObject(aco);
    if (ok) {
      RCLCPP_INFO(get_logger(), "MoveIt attach OK for %s on %s.", name.c_str(), kAttachLink);
    } else {
      RCLCPP_WARN(get_logger(),
                  "MoveIt attach FAILED for %s (Gazebo attach OK); continuing degraded.",
                  name.c_str());
    }
    return ok;
  }

  /**
   * detachObjectMoveIt() — ACO REMOVE（Humble 无 detachCollisionObject），
   * 可选在 Gazebo detach 落地后把物体重新加回场景（放置点位置）。
   * re_add=false 用于 transfer 回退路径（只摘不挂）。
   */
  bool detachObjectMoveIt(const std::string& name,
                          const geometry_msgs::msg::Pose* fallback_pose = nullptr,
                          bool re_add = true) {
    if (!moveitSceneEnabled() || !initPlanningSceneInterface()) {
      return false;
    }
    moveit_msgs::msg::AttachedCollisionObject aco;
    aco.link_name = kAttachLink;
    aco.object.id = name;
    aco.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    const bool removed = planning_scene_interface_->applyAttachedCollisionObject(aco);

    if (re_add && get_parameter_or<bool>("moveit_scene_readd_after_detach", true)) {
      rclcpp::sleep_for(300ms); // 等物体在 Gazebo 落地/稳定
      geometry_msgs::msg::Pose p;
      if (readObjectPose(name, p)) {
        moveit_msgs::msg::CollisionObject co;
        makeObjectCollisionObject(name, p, co);
        planning_scene_interface_->applyCollisionObject(co);
      } else if (fallback_pose) {
        moveit_msgs::msg::CollisionObject co;
        makeObjectCollisionObject(name, *fallback_pose, co);
        planning_scene_interface_->applyCollisionObject(co);
      } else {
        RCLCPP_WARN(get_logger(),
                    "Cannot re-add %s to planning scene; will re-add at next pick refresh.",
                    name.c_str());
      }
    }
    return removed;
  }

  /**
   * applyGripperObjectAcm() — 全量 Allowed Collision Matrix：
   *   - 复刻 SRDF disable_collisions 对（Humble diff 会整体替换 ACM，不复刻则相邻 link
   * 自碰撞被重新打开）
   *   - 豁免夹爪 link（link7/link8/gripper_base/grasp_tcp）× 3 物体
   *     （抓取位姿处张开手指与物体碰撞几何重叠，不豁免则预检/执行必挂）
   */
  bool applyGripperObjectAcm(moveit::planning_interface::MoveGroupInterface& move_group) {
    if (!moveitSceneEnabled() || !initPlanningSceneInterface()) {
      return false;
    }

    auto robot_model = move_group.getRobotModel();
    std::vector<std::string> names = robot_model->getLinkModelNames();
    for (const auto& obj : objects()) {
      names.push_back(obj.name);
    }

    auto key = [](const std::string& a, const std::string& b) {
      return a < b ? std::make_pair(a, b) : std::make_pair(b, a);
    };
    std::set<std::pair<std::string, std::string>> allowed_pairs;
    const auto srdf = robot_model->getSRDF();
    if (srdf) {
      for (const auto& dc : srdf->getDisabledCollisionPairs()) {
        allowed_pairs.insert(key(dc.link1_, dc.link2_));
      }
    }
    for (const char* link : kGripperTouchLinks) {
      for (const auto& obj : objects()) {
        allowed_pairs.insert(key(link, obj.name));
      }
    }

    moveit_msgs::msg::AllowedCollisionMatrix acm;
    acm.entry_names = names;
    acm.entry_values.resize(names.size());
    for (size_t i = 0; i < names.size(); ++i) {
      acm.entry_values[i].enabled.resize(names.size(), false);
      for (size_t j = 0; j < names.size(); ++j) {
        acm.entry_values[i].enabled[j] = allowed_pairs.count(key(names[i], names[j])) > 0;
      }
    }

    moveit_msgs::msg::PlanningScene ps;
    ps.is_diff = true;
    ps.allowed_collision_matrix = acm;
    const bool ok = planning_scene_interface_->applyPlanningScene(ps);
    if (ok) {
      RCLCPP_INFO(get_logger(),
                  "Gripper-object ACM applied: %zu names x %zu entries, "
                  "%zu allowed pairs.",
                  names.size(), names.size(), allowed_pairs.size());
    } else {
      RCLCPP_WARN(get_logger(), "Gripper-object ACM apply FAILED.");
    }
    return ok;
  }

  /**
   * readObjectPosition() — 从 Gazebo 读取物体的当前位置
   *
   *  通过 /get_entity_state 服务获取 Gazebo 场景中物体的位置。
   *  由于物体可能被碰撞移动，必须在每次抓取前重新读取。
   *
   *  @param name   物体的 Gazebo 模型名称
   *  @param point  [输出] 物体的位置（世界坐标系）
   *  @return true=读取成功, false=超时或 Gazebo 返回错误
   */
  bool readObjectPosition(const std::string& name, geometry_msgs::msg::Point& point) {
    if (!get_entity_client_->wait_for_service(5s)) {
      RCLCPP_ERROR(get_logger(), "Gazebo /get_entity_state service is not available.");
      return false;
    }

    auto request = std::make_shared<gazebo_msgs::srv::GetEntityState::Request>();
    request->name = name;
    request->reference_frame = "world";

    auto future = get_entity_client_->async_send_request(request);
    if (future.wait_for(3s) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "Timed out reading Gazebo entity %s.", name.c_str());
      return false;
    }

    const auto response = future.get();
    if (!response->success) {
      RCLCPP_ERROR(get_logger(), "Gazebo could not read state for %s.", name.c_str());
      return false;
    }

    point = response->state.pose.position;
    return true;
  }

  /** 阻塞等待一份新鲜的感知数据（Docker 推理产出 perception_grasps.csv 完成刷新） */
  bool waitForFreshPerceptionTargets(const std::chrono::seconds timeout,
                                     const std::set<std::string>& required_objects = {}) {
    const auto deadline = now() + rclcpp::Duration(timeout);
    rclcpp::Rate rate(2);

    while (rclcpp::ok() && now() < deadline) {
      if (tryRefreshPerceptionCache(required_objects)) {
        return true;
      }
      rate.sleep();
    }

    RCLCPP_ERROR(get_logger(), "Timed out waiting for GraspNet 6D perception_grasps.csv.");
    return false;
  }

  /**
   * tryRefreshPerceptionCache() — 非阻塞刷新感知缓存（单次尝试）
   * 从 perception_grasps.csv 读取一次目标数据；required_objects 非空时
   * 仅要求这些物体的数据新鲜完整（点数≥下限且含 6D 姿态），
   * 全部满足才替换缓存并置 ready 标志（防止半新数据污染）
   */
  bool tryRefreshPerceptionCache(const std::set<std::string>& required_objects = {}) {
    std::map<std::string, std::vector<PerceptionTarget>> targets;
    if (!loadPerceptionTargets(targets)) {
      return false;
    }

    for (const auto& object : objects()) {
      if (!required_objects.empty() && required_objects.count(object.name) == 0) {
        continue;
      }
      const auto key = perceptionKeyForObject(object.name);
      const auto it = targets.find(key);
      if (it == targets.end() || it->second.empty() ||
          it->second.front().count < kMinPerceptionPoints || !it->second.front().has_grasp_pose) {
        return false;
      }
    }

    cached_perception_targets_ = targets;
    perception_cache_ready_ = true;
    RCLCPP_INFO(get_logger(), "Perception cache refreshed: %zu objects from fresh CSV.",
                cached_perception_targets_.size());
    return true;
  }
  /**
   * readPerceptionObjectPosition() — 从感知缓存读取物体的视觉中心位置。
   * 只返回缓存的 object_center_world（不访问 Gazebo），
   * 需配合 readValidatedVisionObjectPosition 做 Gazebo 交叉校验
   */
  bool readPerceptionObjectPosition(const std::string& object_name,
                                    geometry_msgs::msg::Point& point) {
    if (!perception_cache_ready_) {
      RCLCPP_ERROR(get_logger(), "Initial perception target cache is not ready for %s.",
                   object_name.c_str());
      return false;
    }

    const auto key = perceptionKeyForObject(object_name);
    const auto it = cached_perception_targets_.find(key);
    if (it == cached_perception_targets_.end() || it->second.empty()) {
      RCLCPP_ERROR(get_logger(), "Cached perception target %s for object %s is missing.",
                   key.c_str(), object_name.c_str());
      return false;
    }
    const auto& target = it->second.front();
    if (target.count < kMinPerceptionPoints) {
      RCLCPP_ERROR(get_logger(), "Perception target %s has only %d points; need >= %d.",
                   key.c_str(), target.count, kMinPerceptionPoints);
      return false;
    }
    if (!target.has_grasp_pose) {
      RCLCPP_ERROR(get_logger(), "Perception target %s has no camera-recognized 6D grasp pose.",
                   key.c_str());
      return false;
    }
    point = target.object_center_world;
    RCLCPP_INFO(get_logger(),
                "Cached visual object %s: center_camera=(%.3f, %.3f, %.3f) "
                "center_world=(%.3f, %.3f, %.3f), grasp_world=(%.3f, %.3f, %.3f), "
                "q=(%.3f, %.3f, %.3f, %.3f), points=%d",
                object_name.c_str(), target.object_center_camera.x, target.object_center_camera.y,
                target.object_center_camera.z, point.x, point.y, point.z, target.world.x,
                target.world.y, target.world.z, target.orientation.x, target.orientation.y,
                target.orientation.z, target.orientation.w, target.count);
    return true;
  }

  // setInferencePaused removed — GraspNet runs freely

  /**
   * readValidatedVisionObjectPosition() — 视觉位置 + Gazebo 交叉校验。
   * 视觉中心与 Gazebo 真实位置偏差超过 kVisionTargetMaxError（0.08m）
   * 即拒绝 —— 防感知漂移/误检污染抓取坐标，拒绝使用固定坐标或猜测值
   */
  bool readValidatedVisionObjectPosition(const std::string& object_name,
                                         geometry_msgs::msg::Point& point) {
    if (!readPerceptionObjectPosition(object_name, point)) {
      return false;
    }

    geometry_msgs::msg::Point gazebo_point;
    if (!readObjectPosition(object_name, gazebo_point)) {
      RCLCPP_ERROR(get_logger(),
                   "Target validation failed for %s: cannot read Gazebo "
                   "state for verification.",
                   object_name.c_str());
      return false;
    }

    const double dx = gazebo_point.x - point.x;
    const double dy = gazebo_point.y - point.y;
    const double dz = gazebo_point.z - point.z;
    const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
    RCLCPP_INFO(get_logger(),
                "Pre-grasp target validation %s: vision_base=(%.4f, %.4f, %.4f) "
                "gazebo_actual=(%.4f, %.4f, %.4f) delta=(%.4f, %.4f, %.4f) "
                "dist=%.4f m",
                object_name.c_str(), point.x, point.y, point.z, gazebo_point.x, gazebo_point.y,
                gazebo_point.z, dx, dy, dz, dist);

    if (dist > kVisionTargetMaxError) {
      RCLCPP_ERROR(get_logger(),
                   "Target validation failed for %s: visual target error %.4f "
                   "m > %.4f m. Refusing to use fixed coordinates or guess.",
                   object_name.c_str(), dist, kVisionTargetMaxError);
      return false;
    }
    return true;
  }

  /** 位置校验通过后，取出世界系抓取姿态（保留 GraspNet 的 6D 方向预测） */
  bool readValidatedVisionObjectGrasp(const std::string& object_name,
                                      geometry_msgs::msg::Point& point,
                                      geometry_msgs::msg::Quaternion& orientation) {
    if (!readValidatedVisionObjectPosition(object_name, point)) {
      return false;
    }

    const auto key = perceptionKeyForObject(object_name);
    const auto it = cached_perception_targets_.find(key);
    if (it == cached_perception_targets_.end() || it->second.empty() ||
        !it->second.front().has_grasp_pose) {
      RCLCPP_ERROR(get_logger(), "Validated target %s is missing a camera-recognized grasp pose.",
                   object_name.c_str());
      return false;
    }
    const auto& target = it->second.front();
    orientation = target.orientation;
    RCLCPP_INFO(get_logger(),
                "Using camera-recognized grasp pose for %s: q_world=(%.4f, %.4f, %.4f, %.4f), "
                "score=%.4f, width=%.4f, height=%.4f, depth=%.4f",
                object_name.c_str(), orientation.x, orientation.y, orientation.z, orientation.w,
                target.score, target.width, target.height, target.depth);
    return true;
  }

  /** 读取物体的全部有效 6D 候选（有姿态且点数≥下限），供候选遍历器使用 */
  bool readValidatedVisionObjectGraspCandidates(const std::string& object_name,
                                                std::vector<PerceptionTarget>& candidates) {
    const auto key = perceptionKeyForObject(object_name);
    const auto it = cached_perception_targets_.find(key);
    if (it == cached_perception_targets_.end() || it->second.empty()) {
      RCLCPP_ERROR(get_logger(), "Validated target %s is missing GraspNet grasp candidates.",
                   object_name.c_str());
      return false;
    }
    candidates.clear();
    for (const auto& target : it->second) {
      if (target.has_grasp_pose && target.count >= kMinPerceptionPoints) {
        candidates.push_back(target);
      }
    }
    RCLCPP_INFO(get_logger(), "%s has %zu complete GraspNet 6D candidates.", object_name.c_str(),
                candidates.size());
    return !candidates.empty();
  }

  /**
   * verifyObjectNear() — 校验物体实际位置与期望位置偏差在 tolerance 内。
   * 用于 detach 后确认物体落在放置点（容差 0.08m）；不通过仅告警不阻断
   */
  bool verifyObjectNear(const std::string& name, const geometry_msgs::msg::Point& expected,
                        double tolerance, const std::string& label) {
    geometry_msgs::msg::Point actual;
    if (!readObjectPosition(name, actual)) {
      return false;
    }
    const double dx = actual.x - expected.x;
    const double dy = actual.y - expected.y;
    const double dz = actual.z - expected.z;
    const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (dist > tolerance) {
      RCLCPP_ERROR(get_logger(),
                   "%s validation failed for %s: expected=(%.3f, %.3f, %.3f) "
                   "actual=(%.3f, %.3f, %.3f) dist=%.3f > %.3f",
                   label.c_str(), name.c_str(), expected.x, expected.y, expected.z, actual.x,
                   actual.y, actual.z, dist, tolerance);
      return false;
    }
    RCLCPP_INFO(get_logger(),
                "%s validation OK for %s: actual=(%.3f, %.3f, %.3f) "
                "dist=%.3f",
                label.c_str(), name.c_str(), actual.x, actual.y, actual.z, dist);
    return true;
  }

private:
  // ── 感知 CSV 解析 ──
  std::vector<std::string> splitCsvLine(const std::string& line);
  // 读取 perception_grasps.csv → 按物体名分组的 PerceptionTarget 列表
  // （CSV 每条候选一行，字段与 PerceptionTarget 一一对应）
  bool loadPerceptionTargets(std::map<std::string, std::vector<PerceptionTarget>>& targets);

  /**
   * commandGripper() — 向夹爪控制器发送轨迹命令
   *
   *  夹爪由两个关节（joint7 / joint8）构成，对称运动。
   *  通过 FollowJointTrajectory Action 发送单点轨迹，
   *  等待执行完成。
   *
   *  @param joint7  joint7 目标位置（正代表张开）
   *  @param joint8  joint8 目标位置（负代表张开，与 joint7 对称）
   */
  void commandGripper(double joint7, double joint8, double duration_sec) {
    auto goal = control_msgs::action::FollowJointTrajectory::Goal();
    goal.trajectory.joint_names = {"joint7", "joint8"};

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {joint7, joint8};
    point.time_from_start = rclcpp::Duration::from_seconds(duration_sec);
    goal.trajectory.points.push_back(point);

    auto send_goal_future = gripper_action_client_->async_send_goal(goal);
    if (send_goal_future.wait_for(3s) != std::future_status::ready) {
      RCLCPP_WARN(get_logger(), "Gripper action send timed out.");
      return;
    }

    auto goal_handle = send_goal_future.get();
    if (!goal_handle) {
      RCLCPP_WARN(get_logger(), "Gripper action was rejected.");
      return;
    }

    auto result_future = gripper_action_client_->async_get_result(goal_handle);
    if (result_future.wait_for(4s) != std::future_status::ready) {
      RCLCPP_WARN(get_logger(), "Gripper action result timed out.");
      return;
    }

    const auto result = result_future.get();
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_WARN(get_logger(), "Gripper action finished with code %d.",
                  static_cast<int>(result.code));
    }
  }

  /**
   * reportMissingRuntimeDependency() — 诊断输出缺失的运行时依赖
   *
   *  逐一检查四个依赖项，明确告知用户缺少哪个。
   *  注意：此函数假定之前 wait_for 已经超时，因此使用 1ms 的即刻检查。
   */
  void reportMissingRuntimeDependency() {
    if (!gripper_action_client_->wait_for_action_server(1ms)) {
      RCLCPP_ERROR(get_logger(), "Missing /gripper_controller/follow_joint_trajectory "
                                 "action server.");
    }
    if (!get_entity_client_->service_is_ready()) {
      RCLCPP_ERROR(get_logger(), "Missing /get_entity_state service. "
                                 "Make sure Gazebo is running.");
    }
    if (!attach_client_->service_is_ready()) {
      RCLCPP_ERROR(get_logger(), "Missing /piper/attach service. Make sure piper_pick.world "
                                 "loads libpiper_link_attacher.so.");
    }
    if (!detach_client_->service_is_ready()) {
      RCLCPP_ERROR(get_logger(), "Missing /piper/detach service. Make sure piper_pick.world "
                                 "loads libpiper_link_attacher.so.");
    }
  }

  /** 调用 /piper/attach 或 /piper/detach 服务（Gazebo 固定关节吸附/解除） */
  bool callAttachDetach(const rclcpp::Client<piper_gazebo::srv::AttachDetach>::SharedPtr& client,
                        const std::string& operation, const std::string& object_name) {
    if (!client->wait_for_service(5s)) {
      RCLCPP_ERROR(get_logger(), "Missing /piper/%s service.", operation.c_str());
      return false;
    }

    auto request = std::make_shared<piper_gazebo::srv::AttachDetach::Request>();
    request->robot_model = "piper";
    request->robot_link = "link6";
    request->object_model = object_name;
    request->object_link = "link";

    auto future = client->async_send_request(request);
    if (future.wait_for(3s) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "/piper/%s timed out for %s.", operation.c_str(),
                   object_name.c_str());
      return false;
    }

    const auto response = future.get();
    if (!response->success) {
      RCLCPP_ERROR(get_logger(), "/piper/%s failed for %s: %s", operation.c_str(),
                   object_name.c_str(), response->message.c_str());
      return false;
    }

    RCLCPP_INFO(get_logger(), "/piper/%s OK for %s: %s", operation.c_str(), object_name.c_str(),
                response->message.c_str());
    return true;
  }

  // ---- ROS2 客户端成员 ----
  // 全部在构造函数中创建，由 waitForRuntimeDependencies() 统一等待就绪

  /// 夹爪控制 Action Client — 发送 joint7/joint8 轨迹（Gazebo gripper_controller）
  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr
      gripper_action_client_;

  /// Gazebo 物体状态 Service Client — 查询物体实时位置
  rclcpp::Client<gazebo_msgs::srv::GetEntityState>::SharedPtr get_entity_client_;
  /// Gazebo 固定关节吸附 Service Client（libpiper_link_attacher 插件提供）
  rclcpp::Client<piper_gazebo::srv::AttachDetach>::SharedPtr attach_client_;
  /// Gazebo 固定关节解除 Service Client
  rclcpp::Client<piper_gazebo::srv::AttachDetach>::SharedPtr detach_client_;

  /// 初始相机识别结果缓存 — 防止已放置物体进入视野后污染未抓目标中心
  std::map<std::string, std::vector<PerceptionTarget>> cached_perception_targets_;
  bool perception_cache_ready_{false};

  /// MoveIt 规划场景接口（惰性创建，等 move_group 就绪后再初始化）
  std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;

  /// pick_state.json 序号（单调递增，供 Docker 变更检测与调试）
  int pick_state_seq_{0};
};

} // namespace piper_gazebo::pick_place

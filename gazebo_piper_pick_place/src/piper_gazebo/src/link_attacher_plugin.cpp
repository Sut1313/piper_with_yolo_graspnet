/**
 * ============================================================================
 *  piper_gazebo/src/link_attacher_plugin.cpp
 *
 *  【文件概述】
 *  Gazebo World Plugin — 提供 /piper/attach 和 /piper/detach 两个 ROS2 服务，
 *  用于在仿真中实现物体的吸附（固定关节）与分离（销毁固定关节）。
 *
 *  这是在 Gazebo 中实现"抓取"效果的关键组件：
 *  当机械臂末端触碰到物体时，pick_and_place 节点调用 attach 服务，
 *  该插件在 Gazbo 物理引擎层创建一个固定关节将物体 Link 固定在机器人
 *  末端 Link 上，使物体随机械臂一起运动；到达码放位置后调用 detach
 *  删除该关节，物体落在地上。
 *
 *  【执行的数据流】
 *  1. Gazebo 加载世界文件 worlds/piper_pick.world，其中引用了本插件
 *  2. 插件启动，在 /piper/attach 和 /piper/detach 上提供 Service
 *  3. pick_and_place 节点通过 Service 请求 attach/detach
 *  4. 插件解析请求中的模型名称 → 获取 Link 指针 → 创建/销毁 Gazebo 固定关节
 *
 *  【关键概念】
 *    - 固定关节（Fixed Joint）：Gazebo 中两个 Link 之间的刚性连接，
 *      使它们表现为一个刚体
 *    - 线程安全：所有操作在 mutex 保护下进行，防止并发 Service 请求
 *    - 去重：同一对 Link 的重复 attach 请求会被忽略（幂等）
 * ============================================================================
 */

#include <map>
#include <memory>
#include <mutex>
#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <ignition/math/Pose3.hh>
#include <rclcpp/rclcpp.hpp>

#include "piper_gazebo/srv/attach_detach.hpp"

namespace piper_gazebo {

// ============================================================================
//  LinkAttacherPlugin 类
//  继承 Gazebo WorldPlugin，在仿真世界里注册吸附/分离服务
// ============================================================================

/**
 * LinkAttacherPlugin
 *
 *  这是一个 Gazebo WorldPlugin，意味着它在世界加载时自动实例化。
 *  它在 ROS2 命名空间中注册两个 Service：
 *    - /piper/attach：将物体 Link 固定到机器人 Link
 *    - /piper/detach：解除上述固定连接
 *
 *  【持久状态】
 *    attached_joints_：map<key, JointPtr> 记录当前所有活跃的固定关节，
 *    以便 detach 时查找和销毁。
 *
 *  【线程安全】
 *    handleAttach / handleDetach 都通过 mutex_ 互斥，
 *    确保并发请求不会导致 attached_joints_ 状态不一致。
 */
class LinkAttacherPlugin : public gazebo::WorldPlugin
{
public:
  /**
   * Load() — Gazebo WorldPlugin 入口
   *
   *  当 Gazebo 加载包含本插件的世界文件时调用此方法。
   *  在此创建 ROS2 节点和两个 Service，准备处理 attach/detach 请求。
   *
   *  @param world Gazbo 世界指针（用于访问物理引擎和模型）
   *  @param sdf   插件 SDF 参数（当前未使用）
   */
  void Load(gazebo::physics::WorldPtr world, sdf::ElementPtr sdf) override {
    world_ = world;
    node_ = gazebo_ros::Node::Get(sdf);

    // 注册 /piper/attach 服务
    attach_service_ = node_->create_service<AttachDetach>(
        "/piper/attach", [this](const std::shared_ptr<AttachDetach::Request> request,
                                std::shared_ptr<AttachDetach::Response> response) {
          handleAttach(request, response);
        });

    // 注册 /piper/detach 服务
    detach_service_ = node_->create_service<AttachDetach>(
        "/piper/detach", [this](const std::shared_ptr<AttachDetach::Request> request,
                                std::shared_ptr<AttachDetach::Response> response) {
          handleDetach(request, response);
        });

    RCLCPP_INFO(node_->get_logger(), "Piper link attacher ready: /piper/attach /piper/detach");
  }

private:
  /// 为了代码简洁，在类内部引用自己定义的 Service 类型
  using AttachDetach = piper_gazebo::srv::AttachDetach;

  // ── 私有辅助函数 ──────────────────────────────────────────────────────

  /**
   * keyFor() — 生成 attach 关系的唯一键
   *
   *  格式："机器人模型::机器人Link->物体模型::物体Link"
   *  用于在 attached_joints_ 字典中索引和去重。
   *
   *  示例："piper::link6->green_cylinder::link"
   */
  static std::string keyFor(const AttachDetach::Request& request) {
    return request.robot_model + "::" + request.robot_link + "->" + request.object_model +
           "::" + request.object_link;
  }

  /**
   * getLink() — 通过模型名称和 Link 名称获取 Gazebo Link 指针
   *
   *  在 Gazebo 世界中查找指定模型的指定 Link。
   *  如果模型或 Link 不存在，通过 message 参数返回错误描述。
   *
   *  @param model_name  Gazebo 模型名称
   *  @param link_name   Link 名称
   *  @param message     [输出] 错误描述字符串
   *  @return 找到则返回 LinkPtr，否则返回 nullptr
   */
  gazebo::physics::LinkPtr getLink(const std::string& model_name, const std::string& link_name,
                                   std::string& message) {
    if (!world_) {
      message = "Gazebo world is not available.";
      return nullptr;
    }

    auto model = world_->ModelByName(model_name);
    if (!model) {
      message = "Model not found: " + model_name;
      return nullptr;
    }

    auto link = model->GetLink(link_name);
    if (!link) {
      message = "Link not found: " + model_name + "::" + link_name;
      return nullptr;
    }
    return link;
  }

  // ── Service 回调 ──────────────────────────────────────────────────────

  /**
   * handleAttach() — /piper/attach 服务回调
   *
   *  完整流程：
   *  1. 加锁（mutex_）
   *  2. 检查是否已经 attach（幂等处理）
   *  3. 查找机器人 Link 和物体 Link
   *  4. 在机器人模型上创建固定关节（Fixed Joint）
   *  5. Attach 两个 Link，Load，Init
   *  6. 记录到 attached_joints_ 字典
   *
   *  注意：创建固定关节时使用机器人的模型（robot_link->GetModel()）
   *  作为关节的父级，这是 Gazebo 关节管理的约定。
   */
  void handleAttach(const std::shared_ptr<AttachDetach::Request> request,
                    std::shared_ptr<AttachDetach::Response> response) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto key = keyFor(*request);
    if (attached_joints_.count(key) > 0) {
      response->success = true;
      response->message = "Already attached: " + key;
      return;
    }

    std::string message;
    auto robot_link = getLink(request->robot_model, request->robot_link, message);
    if (!robot_link) {
      response->success = false;
      response->message = message;
      return;
    }
    auto object_link = getLink(request->object_model, request->object_link, message);
    if (!object_link) {
      response->success = false;
      response->message = message;
      return;
    }

    auto physics = world_->Physics();
    if (!physics) {
      response->success = false;
      response->message = "Gazebo physics engine is not available.";
      return;
    }

    // 创建固定关节：以机器人 Link 所在模型为父级
    auto joint = physics->CreateJoint("fixed", robot_link->GetModel());
    if (!joint) {
      response->success = false;
      response->message = "Failed to create fixed joint.";
      return;
    }

    joint->SetName("piper_attached_" + request->object_model);
    joint->Attach(robot_link, object_link);
    joint->Load(robot_link, object_link, ignition::math::Pose3d::Zero);
    joint->Init();

    attached_joints_[key] = joint;
    response->success = true;
    response->message = "Attached " + key;
    RCLCPP_INFO(node_->get_logger(), "%s", response->message.c_str());
  }

  /**
   * handleDetach() — /piper/detach 服务回调
   *
   *  流程：
   *  1. 加锁
   *  2. 在 attached_joints_ 字典中查找对应的关节
   *  3. 如果找不到（可能已经 detach 或从未 attach），返回成功（幂等）
   *  4. 调用 Detach() 断开关节
   *  5. 调用 Fini() 终结关节
   *  6. 从字典中移除记录
   */
  void handleDetach(const std::shared_ptr<AttachDetach::Request> request,
                    std::shared_ptr<AttachDetach::Response> response) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto key = keyFor(*request);
    auto found = attached_joints_.find(key);
    if (found == attached_joints_.end()) {
      response->success = true;
      response->message = "No active attachment for " + key;
      return;
    }

    // 销毁固定关节
    found->second->Detach();
    found->second->Fini();
    attached_joints_.erase(found);

    response->success = true;
    response->message = "Detached " + key;
    RCLCPP_INFO(node_->get_logger(), "%s", response->message.c_str());
  }

  // ── 成员变量 ──────────────────────────────────────────────────────────

  /// Gazebo 世界指针（用于访问模型和物理引擎）
  gazebo::physics::WorldPtr world_;

  /// ROS2 节点（通过 gazebo_ros::Node::Get 创建）
  gazebo_ros::Node::SharedPtr node_;

  /// /piper/attach 服务
  rclcpp::Service<AttachDetach>::SharedPtr attach_service_;

  /// /piper/detach 服务
  rclcpp::Service<AttachDetach>::SharedPtr detach_service_;

  /// 线程锁 — 保护 attached_joints_ 的并发访问
  std::mutex mutex_;

  /**
   * 当前活跃的吸附关节映射表
   *  key  = "robot_model::robot_link->object_model::object_link"
   *  value = Gazebo Joint 指针
   *
   *  由于 Gazebo 支持同时附着多个物体，因此采用 map 来管理。
   */
  std::map<std::string, gazebo::physics::JointPtr> attached_joints_;
};

// ── 注册 Gazebo WorldPlugin ──
// 这个宏使 Gazebo 在加载世界文件时能够通过插件名称找到本类。
GZ_REGISTER_WORLD_PLUGIN(LinkAttacherPlugin)

} // namespace piper_gazebo

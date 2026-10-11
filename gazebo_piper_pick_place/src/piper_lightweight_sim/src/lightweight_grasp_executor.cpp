#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <future>
#include <map>
#include <mutex>
#include <random>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

using namespace std::chrono_literals;

namespace
{
constexpr double kToolToGraspOffset = 0.138;
// This is wall-clock time.  A 4 s simulated gripper trajectory can take much
// longer while Gazebo, CPU ONNX inference and GraspNet share one workstation.
constexpr auto kGripperWallTimeout = 120s;

struct Candidate
{
  std::string name;
  Eigen::Vector3d grasp_camera;
  Eigen::Matrix3d rotation_camera;
  Eigen::Vector3d object_center_camera;
  double score{0.0};
  double width{0.06};
  long frame_sequence{0};
  double timestamp{0.0};
};

struct ObjectSpec
{
  std::string name;
  int primitive_type;
  std::vector<double> dimensions;
  Eigen::Vector3d initial_center;
  Eigen::Vector3d place_center;
  double fallback_width;
};

struct GraspPlan
{
  Candidate candidate;
  geometry_msgs::msg::Pose approach;
  geometry_msgs::msg::Pose grasp;
  geometry_msgs::msg::Pose lift;
  geometry_msgs::msg::Pose place;
  Eigen::Vector3d object_world;
  Eigen::Vector3d placed_object_world;
  double closing_width{0.05};
  moveit::planning_interface::MoveGroupInterface::Plan approach_trajectory;
  bool has_approach_trajectory{false};
  std::map<std::string, double> place_joint_target;
};

enum class AttemptResult {kFailed, kPreflightOnly, kPlaced};
using IkClient = rclcpp::Client<moveit_msgs::srv::GetPositionIK>;

const std::vector<ObjectSpec> & objectSpecs()
{
  static const std::vector<ObjectSpec> specs{
    {"cube", shape_msgs::msg::SolidPrimitive::BOX, {0.075, 0.075, 0.075},
      {0.54, 0.08, 0.2904858}, {0.42, -0.05, 0.2904858}, 0.075},
    {"cuboid", shape_msgs::msg::SolidPrimitive::BOX, {0.11, 0.06, 0.07},
      {0.68, 0.19, 0.2884858}, {0.55, -0.05, 0.2884858}, 0.060},
    {"cylinder", shape_msgs::msg::SolidPrimitive::CYLINDER, {0.085, 0.032},
      {0.56, 0.31, 0.2954858}, {0.45, 0.05, 0.2954858}, 0.064},
  };
  return specs;
}

const ObjectSpec * findSpec(const std::string & name)
{
  const auto & specs = objectSpecs();
  const auto found = std::find_if(
    specs.begin(), specs.end(), [&name](const auto & spec) {return spec.name == name;});
  return found == specs.end() ? nullptr : &*found;
}

std::vector<double> planningHomeForObject(const std::string & name)
{
  // Same non-singular ready posture and per-object joint1 seeding strategy as
  // the upstream project.  All three lightweight-scene objects have positive
  // world y, so their seeds must also turn joint1 in the positive direction.
  double joint1 = 0.0;
  if (name == "cube") {
    joint1 = 0.15;
  } else if (name == "cuboid") {
    joint1 = 0.27;
  } else if (name == "cylinder") {
    joint1 = 0.45;
  }
  return {joint1, 0.55, -0.75, 0.0, 0.55, 0.0};
}

std::vector<std::string> split(const std::string & line)
{
  std::vector<std::string> fields;
  std::stringstream stream(line);
  std::string field;
  while (std::getline(stream, field, ',')) {
    fields.push_back(field);
  }
  return fields;
}

double wallTimeSeconds()
{
  return std::chrono::duration<double>(
    std::chrono::system_clock::now().time_since_epoch()).count();
}

std::vector<Candidate> readCandidates(const std::string & path)
{
  std::ifstream input(path);
  std::vector<Candidate> result;
  std::string line;
  std::getline(input, line);
  while (std::getline(input, line)) {
    const auto fields = split(line);
    if (fields.size() < 24) {
      continue;
    }
    try {
      Candidate candidate;
      candidate.name = fields[0];
      candidate.grasp_camera = {std::stod(fields[1]), std::stod(fields[2]), std::stod(fields[3])};
      candidate.score = std::stod(fields[5]);
      candidate.width = std::stod(fields[6]);
      for (int index = 0; index < 9; ++index) {
        candidate.rotation_camera(index / 3, index % 3) = std::stod(fields[9 + index]);
      }
      candidate.object_center_camera = {
        std::stod(fields[18]), std::stod(fields[19]), std::stod(fields[20])};
      if (fields.size() >= 26) {
        candidate.frame_sequence = std::stol(fields[24]);
        candidate.timestamp = std::stod(fields[25]);
      }
      result.push_back(candidate);
    } catch (const std::exception &) {
      continue;
    }
  }
  return result;
}

geometry_msgs::msg::Pose poseFromEigen(
  const Eigen::Vector3d & position, const Eigen::Matrix3d & rotation)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = position.x();
  pose.position.y = position.y();
  pose.position.z = position.z();
  Eigen::Quaterniond quaternion(rotation);
  quaternion.normalize();
  pose.orientation.x = quaternion.x();
  pose.orientation.y = quaternion.y();
  pose.orientation.z = quaternion.z();
  pose.orientation.w = quaternion.w();
  return pose;
}

double positionDistance(
  const geometry_msgs::msg::Point & first, const geometry_msgs::msg::Point & second)
{
  const double dx = first.x - second.x;
  const double dy = first.y - second.y;
  const double dz = first.z - second.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool samePose(
  const geometry_msgs::msg::Pose & first, const geometry_msgs::msg::Pose & second)
{
  if (positionDistance(first.position, second.position) > 1e-5) {
    return false;
  }
  const Eigen::Quaterniond first_q(
    first.orientation.w, first.orientation.x, first.orientation.y, first.orientation.z);
  const Eigen::Quaterniond second_q(
    second.orientation.w, second.orientation.x, second.orientation.y, second.orientation.z);
  return std::abs(first_q.normalized().dot(second_q.normalized())) > 1.0 - 1e-7;
}

bool hasFastIk(
  const rclcpp::Node::SharedPtr & node,
  const IkClient::SharedPtr & client,
  const moveit_msgs::msg::RobotState & seed,
  const geometry_msgs::msg::Pose & pose,
  double timeout_seconds)
{
  if (!client) {
    return true;
  }
  auto request = std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();
  request->ik_request.group_name = "arm";
  request->ik_request.robot_state = seed;
  request->ik_request.avoid_collisions = true;
  request->ik_request.ik_link_name = "tool0";
  request->ik_request.pose_stamped.header.frame_id = "world";
  request->ik_request.pose_stamped.header.stamp = node->now();
  request->ik_request.pose_stamped.pose = pose;
  request->ik_request.timeout = rclcpp::Duration::from_seconds(timeout_seconds);
  auto future = client->async_send_request(request);
  const auto wall_timeout = std::chrono::duration<double>(timeout_seconds + 0.20);
  if (future.wait_for(wall_timeout) != std::future_status::ready) {
    return false;
  }
  return future.get()->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
}

bool fastIkSequence(
  const rclcpp::Node::SharedPtr & node,
  moveit::planning_interface::MoveGroupInterface & arm,
  const IkClient::SharedPtr & client,
  const GraspPlan & candidate,
  double timeout_seconds)
{
  if (!client) {
    return true;
  }
  const auto current = arm.getCurrentState(0.25);
  if (!current) {
    return false;
  }
  moveit_msgs::msg::RobotState seed;
  moveit::core::robotStateToRobotStateMsg(*current, seed);
  if (!hasFastIk(node, client, seed, candidate.approach, timeout_seconds)) {
    return false;
  }
  if (!samePose(candidate.approach, candidate.grasp) &&
    !hasFastIk(node, client, seed, candidate.grasp, timeout_seconds))
  {
    return false;
  }
  return hasFastIk(node, client, seed, candidate.lift, timeout_seconds) &&
         hasFastIk(node, client, seed, candidate.place, timeout_seconds);
}

bool planPoseFromState(
  const rclcpp::Node::SharedPtr & node,
  moveit::planning_interface::MoveGroupInterface & arm,
  const geometry_msgs::msg::Pose & pose,
  const std::string & label,
  const moveit::core::RobotState & start_state,
  moveit::planning_interface::MoveGroupInterface::Plan & plan)
{
  arm.setStartState(start_state);
  arm.setPoseTarget(pose, "tool0");
  const bool planned = static_cast<bool>(arm.plan(plan));
  arm.clearPoseTargets();
  if (!planned) {
    RCLCPP_WARN(node->get_logger(), "preflight failed at %s", label.c_str());
  }
  return planned;
}

bool preflightSequence(
  const rclcpp::Node::SharedPtr & node,
  moveit::planning_interface::MoveGroupInterface & arm,
  GraspPlan & candidate)
{
  auto current = arm.getCurrentState(2.0);
  if (!current) {
    RCLCPP_ERROR(node->get_logger(), "cannot read current state for four-stage preflight");
    return false;
  }
  // Preserve the upstream four-way AND semantics: every stage must be
  // independently plannable from the known collision-free planning-home
  // state.  Formal execution still replans grasp/lift/place from their real
  // preceding states; only the approach trajectory is safe to cache here.
  const std::array<std::pair<std::string, geometry_msgs::msg::Pose>, 4> stages{{
    {"approach", candidate.approach},
    {"grasp", candidate.grasp},
    {"lift", candidate.lift},
    {"place", candidate.place},
  }};
  for (const auto & [name, pose] : stages) {
    // Raw GraspNet poses use zero approach clearance, so approach and grasp
    // are exactly the same target.  Checking the same stochastic OMPL query
    // twice adds latency without adding safety.
    if (name == "grasp" && samePose(candidate.approach, candidate.grasp)) {
      continue;
    }
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (!planPoseFromState(node, arm, pose, name, *current, plan)) {
      arm.setStartStateToCurrentState();
      return false;
    }
    // The first stage starts at the real current state, so it is directly
    // executable.  Preserve it instead of asking stochastic OMPL to solve the
    // same query again after preflight has already passed.
    if (name == "approach") {
      candidate.approach_trajectory = plan;
      candidate.has_approach_trajectory = true;
    } else if (name == "place") {
      const auto & trajectory = plan.trajectory_.joint_trajectory;
      if (!trajectory.points.empty()) {
        const auto & positions = trajectory.points.back().positions;
        for (size_t index = 0;
          index < trajectory.joint_names.size() && index < positions.size(); ++index)
        {
          candidate.place_joint_target[trajectory.joint_names[index]] = positions[index];
        }
      }
    }
  }
  arm.setStartStateToCurrentState();
  RCLCPP_INFO(
    node->get_logger(), "four-stage preflight PASS: approach AND grasp AND lift AND place");
  return true;
}

bool planAndExecutePose(
  const rclcpp::Node::SharedPtr & node,
  moveit::planning_interface::MoveGroupInterface & arm,
  const geometry_msgs::msg::Pose & pose,
  const std::string & label,
  bool prefer_cartesian)
{
  arm.setStartStateToCurrentState();
  if (prefer_cartesian) {
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double fraction = arm.computeCartesianPath({pose}, 0.008, 0.0, trajectory);
    if (fraction >= 0.90) {
      if (static_cast<bool>(arm.execute(trajectory))) {
        RCLCPP_INFO(node->get_logger(), "%s executed as Cartesian path", label.c_str());
        return true;
      }
      RCLCPP_WARN(node->get_logger(), "Cartesian execution failed for %s", label.c_str());
      return false;
    }
    RCLCPP_WARN(
      node->get_logger(), "Cartesian path for %s covered %.1f%%; using OMPL fallback",
      label.c_str(), fraction * 100.0);
  }

  arm.setPoseTarget(pose, "tool0");
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const bool planned = static_cast<bool>(arm.plan(plan));
  arm.clearPoseTargets();
  if (!planned) {
    RCLCPP_ERROR(node->get_logger(), "planning failed for %s", label.c_str());
    return false;
  }
  const bool executed = static_cast<bool>(arm.execute(plan));
  if (!executed) {
    RCLCPP_ERROR(node->get_logger(), "execution failed for %s", label.c_str());
  }
  return executed;
}

bool moveToJoints(
  const rclcpp::Node::SharedPtr & node,
  moveit::planning_interface::MoveGroupInterface & arm,
  const std::vector<double> & joints,
  const std::string & label)
{
  arm.setStartStateToCurrentState();
  if (!arm.setJointValueTarget(joints)) {
    return false;
  }
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (!static_cast<bool>(arm.plan(plan))) {
    RCLCPP_WARN(node->get_logger(), "joint planning failed for %s", label.c_str());
    return false;
  }
  return static_cast<bool>(arm.execute(plan));
}

bool planAndExecuteJointTarget(
  const rclcpp::Node::SharedPtr & node,
  moveit::planning_interface::MoveGroupInterface & arm,
  const std::map<std::string, double> & joints,
  const std::string & label)
{
  if (joints.empty()) {
    RCLCPP_ERROR(node->get_logger(), "no cached joint target for %s", label.c_str());
    return false;
  }
  arm.setStartStateToCurrentState();
  if (!arm.setJointValueTarget(joints)) {
    RCLCPP_ERROR(node->get_logger(), "invalid cached joint target for %s", label.c_str());
    return false;
  }
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (!static_cast<bool>(arm.plan(plan))) {
    RCLCPP_ERROR(node->get_logger(), "joint planning failed for %s", label.c_str());
    return false;
  }
  if (!static_cast<bool>(arm.execute(plan))) {
    RCLCPP_ERROR(node->get_logger(), "joint execution failed for %s", label.c_str());
    return false;
  }
  RCLCPP_INFO(node->get_logger(), "%s used cached preflight IK target", label.c_str());
  return true;
}

bool commandGripper(
  const rclcpp::Node::SharedPtr & node,
  const rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr & client,
  double width)
{
  if (!client->wait_for_action_server(5s)) {
    RCLCPP_ERROR(node->get_logger(), "gripper controller action is unavailable");
    return false;
  }
  control_msgs::action::FollowJointTrajectory::Goal goal;
  goal.trajectory.joint_names = {"gripper"};
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = {std::clamp(width, 0.0, 0.1)};
  // The upstream driver sends GripperCtrl commands continuously for one
  // second.  Use the same default wall duration in simulation while keeping
  // it configurable for controller tuning.
  const double motion_time = std::max(
    0.1, node->get_parameter("gripper_motion_time").as_double());
  point.time_from_start = rclcpp::Duration::from_seconds(motion_time);
  goal.trajectory.points.push_back(point);
  auto goal_future = client->async_send_goal(goal);
  if (goal_future.wait_for(5s) != std::future_status::ready) {
    return false;
  }
  auto handle = goal_future.get();
  if (!handle) {
    RCLCPP_ERROR(node->get_logger(), "gripper trajectory was rejected");
    return false;
  }
  auto result_future = client->async_get_result(handle);
  if (result_future.wait_for(kGripperWallTimeout) != std::future_status::ready) {
    RCLCPP_ERROR(
      node->get_logger(), "gripper trajectory exceeded the 120 s wall-clock timeout");
    return false;
  }
  const auto result = result_future.get();
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_ERROR(
      node->get_logger(), "gripper command %.3f failed with result code %d",
      width, static_cast<int>(result.code));
    return false;
  }
  return true;
}

moveit_msgs::msg::CollisionObject makeCollisionObject(
  const ObjectSpec & spec, const Eigen::Vector3d & center)
{
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = "world";
  object.id = spec.name;
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = spec.primitive_type;
  for (const double dimension : spec.dimensions) {
    primitive.dimensions.push_back(dimension);
  }
  geometry_msgs::msg::Pose pose;
  pose.position.x = center.x();
  pose.position.y = center.y();
  pose.position.z = center.z();
  pose.orientation.w = 1.0;
  object.primitives.push_back(primitive);
  object.primitive_poses.push_back(pose);
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  return object;
}

void initializePlanningScene(moveit::planning_interface::PlanningSceneInterface & scene)
{
  moveit_msgs::msg::CollisionObject table;
  table.header.frame_id = "world";
  table.id = "worktable_top";
  shape_msgs::msg::SolidPrimitive table_box;
  table_box.type = shape_msgs::msg::SolidPrimitive::BOX;
  table_box.dimensions = {0.82, 0.68, 0.05};
  geometry_msgs::msg::Pose table_pose;
  table_pose.position.x = 0.66;
  table_pose.position.y = 0.187732;
  table_pose.position.z = 0.2254858;
  table_pose.orientation.w = 1.0;
  table.primitives.push_back(table_box);
  table.primitive_poses.push_back(table_pose);
  table.operation = moveit_msgs::msg::CollisionObject::ADD;

  std::vector<moveit_msgs::msg::CollisionObject> objects{table};
  for (const auto & spec : objectSpecs()) {
    objects.push_back(makeCollisionObject(spec, spec.initial_center));
  }
  scene.applyCollisionObjects(objects);
}

void writePickState(
  const std::string & path,
  const std::vector<std::string> & all_targets,
  const std::set<std::string> & placed,
  std::int64_t version,
  const std::string & phase,
  bool perception_enabled,
  const std::string & active_target = "")
{
  const std::filesystem::path destination(path);
  const auto temporary = destination.string() + ".tmp";
  std::ofstream output(temporary, std::ios::trunc);
  output << "{\n  \"version\": " << version << ",\n  \"timestamp\": "
         << wallTimeSeconds() << ",\n  \"phase\": \"" << phase
         << "\",\n  \"perception_enabled\": "
         << (perception_enabled ? "true" : "false") << ",\n  \"active_target\": ";
  if (active_target.empty()) {
    output << "null";
  } else {
    output << "\"" << active_target << "\"";
  }
  output << ",\n  \"placed\": [";
  bool first = true;
  for (const auto & name : all_targets) {
    if (placed.count(name) == 0) {
      continue;
    }
    output << (first ? "" : ", ") << "\"" << name << "\"";
    first = false;
  }
  output << "],\n  \"pending\": [";
  first = true;
  for (const auto & name : all_targets) {
    if (placed.count(name) > 0) {
      continue;
    }
    output << (first ? "" : ", ") << "\"" << name << "\"";
    first = false;
  }
  output << "]\n}\n";
  output.close();
  std::filesystem::rename(temporary, destination);
}

class AttachmentBridge
{
public:
  explicit AttachmentBridge(const rclcpp::Node::SharedPtr & node)
  : node_(node)
  {
    for (const auto & spec : objectSpecs()) {
      attach_publishers_[spec.name] = node_->create_publisher<std_msgs::msg::Empty>(
        "/grasp/" + spec.name + "/attach", 10);
      detach_publishers_[spec.name] = node_->create_publisher<std_msgs::msg::Empty>(
        "/grasp/" + spec.name + "/detach", 10);
      state_subscriptions_.push_back(node_->create_subscription<std_msgs::msg::String>(
        "/grasp/" + spec.name + "/state", 10,
        [this, name = spec.name](const std_msgs::msg::String::SharedPtr message) {
          {
            std::lock_guard<std::mutex> lock(mutex_);
            states_[name] = message->data;
            seen_.insert(name);
          }
          condition_.notify_all();
        }));
    }
  }

  bool setAttached(const std::string & name, bool attached)
  {
    auto & publishers = attached ? attach_publishers_ : detach_publishers_;
    const auto found = publishers.find(name);
    if (found == publishers.end()) {
      return false;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      seen_.erase(name);
    }
    std_msgs::msg::Empty command;
    for (int index = 0; index < 3; ++index) {
      found->second->publish(command);
      rclcpp::sleep_for(100ms);
    }
    std::unique_lock<std::mutex> lock(mutex_);
    const std::string expected = attached ? "attached" : "detached";
    const bool received = condition_.wait_for(lock, 4s, [this, &name, &expected]() {
      return seen_.count(name) > 0 && states_[name] == expected;
    });
    if (!received) {
      if (!attached) {
        const auto state = states_.find(name);
        if (state == states_.end() || state->second == "detached") {
          RCLCPP_INFO(
            node_->get_logger(), "%s already detached (no state transition published)",
            name.c_str());
          return true;
        }
      }
      RCLCPP_ERROR(
        node_->get_logger(), "no Gazebo confirmation for %s %s",
        attached ? "attach" : "detach", name.c_str());
    }
    return received;
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::map<std::string, rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr> attach_publishers_;
  std::map<std::string, rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr> detach_publishers_;
  std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> state_subscriptions_;
  std::map<std::string, std::string> states_;
  std::set<std::string> seen_;
  std::mutex mutex_;
  std::condition_variable condition_;
};

std::vector<Candidate> waitForFreshCandidates(
  const rclcpp::Node::SharedPtr & node,
  const std::string & path,
  const std::string & target,
  const std::set<std::string> & placed,
  double not_before,
  double timeout_seconds)
{
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(timeout_seconds));
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    const auto all = readCandidates(path);
    bool contains_placed = false;
    std::vector<Candidate> target_candidates;
    for (const auto & candidate : all) {
      if (placed.count(candidate.name) > 0 && candidate.timestamp >= not_before) {
        contains_placed = true;
      }
      if (candidate.name == target && candidate.timestamp >= not_before) {
        target_candidates.push_back(candidate);
      }
    }
    if (!contains_placed && !target_candidates.empty()) {
      RCLCPP_INFO(
        node->get_logger(), "fresh perception frame %ld provides %zu candidates for %s",
        target_candidates.front().frame_sequence, target_candidates.size(), target.c_str());
      return target_candidates;
    }
    rclcpp::sleep_for(250ms);
  }
  RCLCPP_ERROR(node->get_logger(), "timed out waiting for fresh candidates for %s", target.c_str());
  return {};
}

GraspPlan buildPlan(
  const Candidate & candidate,
  const ObjectSpec & spec,
  const Eigen::Matrix3d & world_R_camera,
  const Eigen::Vector3d & world_t_camera,
  const Eigen::Matrix3d & place_rotation,
  double approach_distance,
  double lift_distance,
  double place_clearance)
{
  const Eigen::Matrix3d graspnet_R_tcp = (Eigen::Matrix3d() <<
    0.0, 0.0, 1.0,
    0.0, 1.0, 0.0,
    -1.0, 0.0, 0.0).finished();
  const Eigen::Vector3d grasp_world = world_R_camera * candidate.grasp_camera + world_t_camera;
  const Eigen::Vector3d object_world =
    world_R_camera * candidate.object_center_camera + world_t_camera;
  const Eigen::Matrix3d world_R_tcp =
    world_R_camera * candidate.rotation_camera * graspnet_R_tcp;
  const Eigen::Vector3d forward = world_R_tcp.col(2);
  const Eigen::Vector3d tool_position = grasp_world - forward * kToolToGraspOffset;
  const Eigen::Vector3d approach_position = tool_position - forward * approach_distance;
  const Eigen::Vector3d lift_position = tool_position + Eigen::Vector3d(0, 0, lift_distance);
  // Gazebo's fixed joint preserves the object pose relative to the TCP.
  const Eigen::Vector3d tool_to_object = world_R_tcp.transpose() * (object_world - tool_position);
  // Match upstream placement semantics: x/y are a fixed TCP transfer target,
  // and z reuses the open-gripper lift height.  Do not rotate-compensate x/y;
  // doing that made each grasp orientation produce a different, often
  // unreachable, place target.  The small floor only keeps the attached
  // object safely above the table before detach.
  Eigen::Vector3d place_position(
    spec.place_center.x(), spec.place_center.y(),
    std::max(lift_position.z(), spec.place_center.z() + kToolToGraspOffset + place_clearance));

  GraspPlan plan;
  plan.candidate = candidate;
  plan.approach = poseFromEigen(approach_position, world_R_tcp);
  plan.grasp = poseFromEigen(tool_position, world_R_tcp);
  plan.lift = poseFromEigen(lift_position, world_R_tcp);
  plan.place = poseFromEigen(place_position, place_rotation);
  plan.object_world = object_world;
  plan.placed_object_world = place_position + place_rotation * tool_to_object;
  plan.placed_object_world.z() = spec.place_center.z();
  plan.closing_width = std::clamp(candidate.width - 0.015, 0.015, 0.080);
  return plan;
}

std::vector<Candidate> fallbackCandidates(
  const Candidate & source,
  const Eigen::Matrix3d & world_R_camera,
  const Eigen::Vector3d & world_t_camera,
  const Eigen::Matrix3d & graspnet_R_tcp)
{
  const Eigen::Vector3d object_world =
    world_R_camera * source.object_center_camera + world_t_camera;
  Eigen::Vector2d radial_xy = object_world.head<2>();
  if (radial_xy.norm() < 1e-6) {
    radial_xy = Eigen::Vector2d::UnitX();
  } else {
    radial_xy.normalize();
  }
  const Eigen::Vector3d radial(radial_xy.x(), radial_xy.y(), 0.0);
  std::vector<Candidate> fallbacks;
  // Tilting the tool toward the object moves the wrist inward while preserving
  // a downward approach.  This is important for the cylinder near the outer
  // diagonal of PiPER's workspace, where a perfectly vertical wrist has no IK.
  for (const double tilt_degrees : {0.0, 15.0, 25.0, 35.0}) {
    const double tilt = tilt_degrees * M_PI / 180.0;
    const Eigen::Vector3d forward =
      (std::sin(tilt) * radial + std::cos(tilt) * Eigen::Vector3d(0, 0, -1)).normalized();
    Eigen::Vector3d y_axis(-radial.y(), radial.x(), 0.0);
    Eigen::Vector3d x_axis = y_axis.cross(forward).normalized();
    y_axis = forward.cross(x_axis).normalized();
    for (const double yaw_degrees : {0.0, 90.0}) {
      const double yaw = yaw_degrees * M_PI / 180.0;
      Eigen::Matrix3d world_R_tcp;
      world_R_tcp.col(0) = std::cos(yaw) * x_axis + std::sin(yaw) * y_axis;
      world_R_tcp.col(1) = -std::sin(yaw) * x_axis + std::cos(yaw) * y_axis;
      world_R_tcp.col(2) = forward;
      Candidate fallback = source;
      fallback.score = -1.0 - tilt_degrees * 0.001 - yaw_degrees * 0.00001;
      fallback.grasp_camera = source.object_center_camera;
      fallback.rotation_camera =
        world_R_camera.transpose() * world_R_tcp * graspnet_R_tcp.transpose();
      fallbacks.push_back(fallback);
    }
  }
  return fallbacks;
}

bool selectPlan(
  const rclcpp::Node::SharedPtr & node,
  moveit::planning_interface::MoveGroupInterface & arm,
  const std::vector<Candidate> & candidates,
  const ObjectSpec & spec,
  const Eigen::Matrix3d & world_R_camera,
  const Eigen::Vector3d & world_t_camera,
  const Eigen::Matrix3d & place_rotation,
  double approach_distance,
  double lift_distance,
  double place_clearance,
  double fallback_approach_distance,
  int max_candidates,
  int max_prefilter_candidates,
  double max_tool_radius,
  const IkClient::SharedPtr & ik_client,
  double fast_ik_timeout,
  bool enable_fallback,
  GraspPlan & selected)
{
  const Eigen::Matrix3d graspnet_R_tcp = (Eigen::Matrix3d() <<
    0.0, 0.0, 1.0,
    0.0, 1.0, 0.0,
    -1.0, 0.0, 0.0).finished();
  std::vector<const Candidate *> ranked;
  ranked.reserve(candidates.size());
  for (const auto & candidate : candidates) {
    ranked.push_back(&candidate);
  }
  const auto reachability_rank = [&](const Candidate * candidate) {
      const Eigen::Vector3d forward =
        (world_R_camera * candidate->rotation_camera * graspnet_R_tcp).col(2);
      const Eigen::Vector3d grasp_world =
        world_R_camera * candidate->grasp_camera + world_t_camera;
      const Eigen::Vector2d radial = grasp_world.head<2>().normalized();
      const double inward_tool_bonus = forward.head<2>().dot(radial);
      const double tool_radius =
        (grasp_world - forward * kToolToGraspOffset).head<2>().norm();
      const double outer_workspace_penalty = std::max(0.0, tool_radius - 0.60);
      // At the outer edge of PiPER's workspace, a high raw GraspNet score is
      // less useful than an approach axis which moves the TCP toward the arm.
      return candidate->score + 2.0 * inward_tool_bonus - 4.0 * outer_workspace_penalty;
    };
  std::stable_sort(
    ranked.begin(), ranked.end(),
    [&](const Candidate * first, const Candidate * second) {
      return reachability_rank(first) > reachability_rank(second);
    });

  // Match the upstream grasp planner's orientation-diversity pass.  GraspNet
  // often returns a large cluster of near-identical high-score poses; trying
  // those consecutively wastes the whole candidate budget on one unreachable
  // wrist configuration.  Canonicalize quaternion sign (q == -q), distribute
  // the ranked candidates into eight orientation buckets, then round-robin.
  std::array<std::vector<const Candidate *>, 8> orientation_buckets;
  for (const Candidate * candidate : ranked) {
    Eigen::Quaterniond q(world_R_camera * candidate->rotation_camera * graspnet_R_tcp);
    q.normalize();
    if (q.w() < 0.0) {
      q.coeffs() *= -1.0;
    }
    int bucket = 0;
    if (q.x() >= 0.0) {bucket |= 1;}
    if (q.y() >= 0.0) {bucket |= 2;}
    if (q.z() >= 0.0) {bucket |= 4;}
    orientation_buckets[static_cast<size_t>(bucket)].push_back(candidate);
  }
  ranked.clear();
  std::array<size_t, 8> bucket_offsets{};
  while (ranked.size() < candidates.size()) {
    bool appended = false;
    for (size_t bucket = 0; bucket < orientation_buckets.size(); ++bucket) {
      if (bucket_offsets[bucket] < orientation_buckets[bucket].size()) {
        ranked.push_back(orientation_buckets[bucket][bucket_offsets[bucket]++]);
        appended = true;
      }
    }
    if (!appended) {
      break;
    }
  }

  // Suppress near-identical world-space poses left after GraspNet NMS.  The
  // old quaternion-sign buckets diversified coarse octants but still allowed
  // dozens of poses from the same local cluster to consume the OMPL budget.
  std::vector<const Candidate *> unique_ranked;
  unique_ranked.reserve(ranked.size());
  constexpr double kNmsPosition = 0.015;
  constexpr double kNmsAngle = 10.0 * M_PI / 180.0;
  for (const Candidate * candidate : ranked) {
    bool duplicate = false;
    for (const Candidate * kept : unique_ranked) {
      if ((candidate->grasp_camera - kept->grasp_camera).norm() >= kNmsPosition) {
        continue;
      }
      const Eigen::Matrix3d delta =
        candidate->rotation_camera.transpose() * kept->rotation_camera;
      const double cosine = std::clamp((delta.trace() - 1.0) * 0.5, -1.0, 1.0);
      if (std::acos(cosine) < kNmsAngle) {
        duplicate = true;
        break;
      }
    }
    if (!duplicate) {
      unique_ranked.push_back(candidate);
    }
  }
  ranked.swap(unique_ranked);

  int prefilter_checked = 0;
  int ik_rejected = 0;
  int planned = 0;
  for (const Candidate * candidate_ptr : ranked) {
    const auto & candidate = *candidate_ptr;
    const Eigen::Matrix3d world_R_tcp =
      world_R_camera * candidate.rotation_camera * graspnet_R_tcp;
    if (world_R_tcp.col(2).z() > -0.30) {
      continue;
    }
    const Eigen::Vector3d grasp_world =
      world_R_camera * candidate.grasp_camera + world_t_camera;
    const Eigen::Vector3d tool_world =
      grasp_world - world_R_tcp.col(2) * kToolToGraspOffset;
    if (tool_world.head<2>().norm() > max_tool_radius) {
      continue;
    }
    if (prefilter_checked++ >= max_prefilter_candidates || planned >= max_candidates) {
      break;
    }
    auto plan = buildPlan(
      candidate, spec, world_R_camera, world_t_camera, place_rotation,
      approach_distance, lift_distance, place_clearance);
    if (!fastIkSequence(node, arm, ik_client, plan, fast_ik_timeout)) {
      ++ik_rejected;
      continue;
    }
    ++planned;
    if (preflightSequence(node, arm, plan)) {
      selected = plan;
      RCLCPP_INFO(
        node->get_logger(),
        "selected %s candidate score=%.3f after fast_ik=%d rejected=%d ompl=%d",
        spec.name.c_str(), candidate.score, prefilter_checked, ik_rejected, planned);
      return true;
    }
  }

  if (!enable_fallback || candidates.empty()) {
    return false;
  }
  RCLCPP_INFO(
    node->get_logger(), "%s raw candidates exhausted: fast_ik=%d rejected=%d ompl=%d; trying fallback",
    spec.name.c_str(), prefilter_checked, ik_rejected, planned);
  const Candidate & source = ranked.empty() ? candidates.front() : *ranked.front();
  int fallback_checked = 0;
  const auto fallbacks = fallbackCandidates(
    source, world_R_camera, world_t_camera, graspnet_R_tcp);
  for (const auto & fallback_candidate : fallbacks) {
    auto fallback = buildPlan(
      fallback_candidate, spec, world_R_camera, world_t_camera, place_rotation,
      fallback_approach_distance, lift_distance, place_clearance);
    fallback.closing_width = spec.fallback_width;
    ++fallback_checked;
    if (!fastIkSequence(node, arm, ik_client, fallback, fast_ik_timeout)) {
      continue;
    }
    if (!preflightSequence(node, arm, fallback)) {
      continue;
    }
    selected = fallback;
    RCLCPP_WARN(
      node->get_logger(), "using tilted fallback %d/%zu for %s",
      fallback_checked, fallbacks.size(),
      spec.name.c_str());
    return true;
  }
  return false;
}

AttemptResult executeObject(
  const rclcpp::Node::SharedPtr & node,
  moveit::planning_interface::MoveGroupInterface & arm,
  moveit::planning_interface::PlanningSceneInterface & scene,
  AttachmentBridge & attachment,
  const rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr & gripper,
  const ObjectSpec & spec,
  const GraspPlan & plan,
  bool dry_run,
  double pose_gate)
{
  if (dry_run) {
    return AttemptResult::kPreflightOnly;
  }
  if (!commandGripper(node, gripper, 0.10)) {
    return AttemptResult::kFailed;
  }
  const bool approach_ok = plan.has_approach_trajectory ?
    static_cast<bool>(arm.execute(plan.approach_trajectory)) :
    planAndExecutePose(node, arm, plan.approach, spec.name + " approach", false);
  if (!approach_ok) {
    RCLCPP_ERROR(node->get_logger(), "execution failed for cached %s approach", spec.name.c_str());
    return AttemptResult::kFailed;
  }
  RCLCPP_INFO(node->get_logger(), "%s approach used the preflight trajectory", spec.name.c_str());
  if (positionDistance(plan.approach.position, plan.grasp.position) <= 1e-4) {
    // Upstream GraspNet mode uses zero 6D approach clearance, so the cached
    // approach trajectory already ends at the exact grasp pose.
    RCLCPP_INFO(
      node->get_logger(), "%s grasp coincides with zero-clearance approach", spec.name.c_str());
  } else if (!planAndExecutePose(node, arm, plan.grasp, spec.name + " grasp", true)) {
    return AttemptResult::kFailed;
  }
  const auto actual_grasp = arm.getCurrentPose("tool0").pose;
  if (positionDistance(actual_grasp.position, plan.grasp.position) > pose_gate) {
    RCLCPP_ERROR(node->get_logger(), "%s rejected by pre-attach TCP distance gate", spec.name.c_str());
    return AttemptResult::kFailed;
  }
  if (!commandGripper(node, gripper, plan.closing_width)) {
    return AttemptResult::kFailed;
  }
  if (!attachment.setAttached(spec.name, true)) {
    commandGripper(node, gripper, 0.10);
    return AttemptResult::kFailed;
  }
  scene.applyCollisionObject(makeCollisionObject(spec, plan.object_world));
  arm.attachObject(
    spec.name, "tool0",
    {"tool0", "link6", "flange_link", "gripper_base", "gripper_link", "gripper_link1", "gripper_link2"});
  rclcpp::sleep_for(200ms);
  // Plan from the actual post-grasp state.  A Cartesian trajectory can lag
  // badly under attached-object load in Gazebo, while this short OMPL move is
  // both faster and already covered by the four-stage preflight.
  if (!planAndExecutePose(node, arm, plan.lift, spec.name + " lift", false)) {
    arm.detachObject(spec.name);
    scene.removeCollisionObjects({spec.name});
    attachment.setAttached(spec.name, false);
    commandGripper(node, gripper, 0.10);
    return AttemptResult::kFailed;
  }
  rclcpp::sleep_for(500ms);

  if (!planAndExecuteJointTarget(
      node, arm, plan.place_joint_target, spec.name + " place"))
  {
    arm.detachObject(spec.name);
    scene.removeCollisionObjects({spec.name});
    attachment.setAttached(spec.name, false);
    commandGripper(node, gripper, 0.10);
    return AttemptResult::kFailed;
  }
  if (!attachment.setAttached(spec.name, false)) {
    arm.detachObject(spec.name);
    scene.removeCollisionObjects({spec.name});
    return AttemptResult::kFailed;
  }
  arm.detachObject(spec.name);
  scene.removeCollisionObjects({spec.name});
  if (!commandGripper(node, gripper, 0.10)) {
    RCLCPP_ERROR(node->get_logger(), "cannot release %s", spec.name.c_str());
    return AttemptResult::kFailed;
  }
  // The fingers are intentionally touching the object at release.  Retreat
  // before inserting the placed object as a MoveIt obstacle; otherwise the
  // next home plan starts in a synthetic object/finger collision.
  auto release_retreat = plan.place;
  release_retreat.position.z += 0.08;
  if (!planAndExecutePose(
      node, arm, release_retreat, spec.name + " release retreat", false))
  {
    RCLCPP_ERROR(node->get_logger(), "cannot retreat after releasing %s", spec.name.c_str());
    return AttemptResult::kFailed;
  }
  rclcpp::sleep_for(300ms);
  scene.applyCollisionObject(makeCollisionObject(spec, plan.placed_object_world));
  RCLCPP_INFO(node->get_logger(), "%s grasped, transferred, and placed", spec.name.c_str());
  return AttemptResult::kPlaced;
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
  const auto node = rclcpp::Node::make_shared("lightweight_grasp_executor", options);

  const std::string shared_dir = node->declare_parameter<std::string>(
    "shared_dir", "shared_camera_data");
  const std::string csv_path = node->declare_parameter<std::string>(
    "perception_grasps_path", shared_dir + "/perception_grasps.csv");
  const std::string state_path = node->declare_parameter<std::string>(
    "pick_state_path", shared_dir + "/pick_state.json");
  const std::string single_target = node->declare_parameter<std::string>("target", "");
  const bool dry_run = node->declare_parameter<bool>("dry_run", false);
  const bool randomize = node->declare_parameter<bool>("randomize_pick_order", true);
  const bool enable_fallback = node->declare_parameter<bool>("enable_position_only_fallback", true);
  const int max_candidates = node->declare_parameter<int>("max_candidates", 10);
  const int max_prefilter_candidates =
    node->declare_parameter<int>("max_prefilter_candidates", 60);
  const int max_rounds = node->declare_parameter<int>("max_rounds", 3);
  const double candidate_planning_time =
    node->declare_parameter<double>("candidate_planning_time", 0.5);
  const double fast_ik_timeout = node->declare_parameter<double>("fast_ik_timeout", 0.08);
  const double execution_planning_time =
    node->declare_parameter<double>("execution_planning_time", 10.0);
  const double velocity_scale = node->declare_parameter<double>("velocity_scale", 0.10);
  const double acceleration_scale =
    node->declare_parameter<double>("acceleration_scale", 1.0);
  const double gripper_motion_time =
    node->declare_parameter<double>("gripper_motion_time", 1.0);
  const double perception_timeout = node->declare_parameter<double>("perception_timeout", 120.0);
  // Match the upstream split semantics: raw GraspNet 6D poses use no extra
  // approach offset; only the isolated top-down position fallback backs off.
  const double approach_distance = node->declare_parameter<double>("approach_distance", 0.0);
  const double fallback_approach_distance =
    node->declare_parameter<double>("fallback_approach_distance", 0.09);
  const double lift_distance = node->declare_parameter<double>("lift_distance", 0.07);
  const double place_clearance = node->declare_parameter<double>("place_clearance", 0.03);
  const double max_tool_radius = node->declare_parameter<double>("max_tool_radius", 0.76);
  const double tcp_pose_gate = node->declare_parameter<double>("tcp_pose_gate", 0.04);

  std::vector<std::string> targets;
  if (!single_target.empty()) {
    if (!findSpec(single_target)) {
      RCLCPP_ERROR(node->get_logger(), "unknown target: %s", single_target.c_str());
      rclcpp::shutdown();
      return 2;
    }
    targets.push_back(single_target);
  } else {
    for (const auto & spec : objectSpecs()) {
      targets.push_back(spec.name);
    }
  }
  if (randomize) {
    std::shuffle(targets.begin(), targets.end(), std::mt19937(std::random_device{}()));
  }

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() {executor.spin();});
  auto shutdown = [&]() {
      executor.cancel();
      if (spin_thread.joinable()) {spin_thread.join();}
      rclcpp::shutdown();
    };

  moveit::planning_interface::MoveGroupInterface arm(node, "arm");
  arm.setPoseReferenceFrame("world");
  arm.setEndEffectorLink("tool0");
  arm.setPlanningTime(execution_planning_time);
  arm.setNumPlanningAttempts(4);
  arm.setGoalPositionTolerance(0.012);
  arm.setGoalOrientationTolerance(0.06);
  // gz_ros2_control's position interface follows fast trajectories with a
  // noticeable lag.  The upstream runtime uses a 0.10 velocity request and a
  // 1.0 acceleration request. Keep the final goal tolerance at 0.05 rad.
  arm.setMaxVelocityScalingFactor(std::clamp(velocity_scale, 0.01, 1.0));
  arm.setMaxAccelerationScalingFactor(std::clamp(acceleration_scale, 0.01, 1.0));
  RCLCPP_INFO(
    node->get_logger(),
    "upstream speed profile: velocity=%.2f acceleration=%.2f precheck=%.2fs exec=%.1fs gripper=%.1fs",
    velocity_scale, acceleration_scale, candidate_planning_time,
    execution_planning_time, gripper_motion_time);
  moveit::planning_interface::PlanningSceneInterface scene;
  initializePlanningScene(scene);
  rclcpp::sleep_for(1s);

  IkClient::SharedPtr ik_client =
    node->create_client<moveit_msgs::srv::GetPositionIK>("/compute_ik");
  if (!ik_client->wait_for_service(5s)) {
    RCLCPP_WARN(node->get_logger(), "compute_ik unavailable; falling back to OMPL-only screening");
    ik_client.reset();
  }

  AttachmentBridge attachment(node);
  const auto gripper = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
    node, "/gripper_controller/follow_joint_trajectory");

  tf2_ros::Buffer tf_buffer(node->get_clock());
  tf_buffer.setUsingDedicatedThread(true);
  tf2_ros::TransformListener tf_listener(tf_buffer, node, false);
  geometry_msgs::msg::TransformStamped transform;
  std::string transform_error;
  const auto transform_deadline = std::chrono::steady_clock::now() + 20s;
  while (rclcpp::ok() && std::chrono::steady_clock::now() < transform_deadline) {
    try {
      transform = tf_buffer.lookupTransform(
        "world", "camera_depth_optical_frame", tf2::TimePointZero);
      break;
    } catch (const std::exception & error) {
      transform_error = error.what();
      rclcpp::sleep_for(250ms);
    }
  }
  if (transform.header.frame_id.empty()) {
    RCLCPP_ERROR(node->get_logger(), "camera TF unavailable: %s", transform_error.c_str());
    shutdown();
    return 3;
  }
  const auto & q_msg = transform.transform.rotation;
  const Eigen::Matrix3d world_R_camera =
    Eigen::Quaterniond(q_msg.w, q_msg.x, q_msg.y, q_msg.z).normalized().toRotationMatrix();
  const Eigen::Vector3d world_t_camera(
    transform.transform.translation.x,
    transform.transform.translation.y,
    transform.transform.translation.z);

  const std::vector<double> ready_joints{0.0, 0.55, -0.75, 0.0, 0.55, 0.0};
  // Camera-clear observation posture validated in the lightweight scene.  At
  // zero joints the arm stays along the lower image edge and all three table
  // objects remain fully visible.  Perception must happen here, not at an
  // object-specific planning home that can place the wrist over its target.
  const std::vector<double> observation_joints{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  const std::vector<double> zero_joints{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::set<std::string> placed;
  std::set<std::string> completed;
  // Epoch-based start keeps perception request IDs unique across executor
  // restarts while the YOLO bridge remains alive.
  std::int64_t state_version = static_cast<std::int64_t>(wallTimeSeconds() * 1000.0);
  writePickState(state_path, targets, placed, state_version, "initializing", false);
  double state_written_at = wallTimeSeconds();

  if (!moveToJoints(node, arm, ready_joints, "initial planning home")) {
    RCLCPP_ERROR(node->get_logger(), "cannot reach initial planning home");
    shutdown();
    return 4;
  }
  if (!commandGripper(node, gripper, 0.10)) {
    RCLCPP_ERROR(node->get_logger(), "cannot initialize the gripper");
    shutdown();
    return 5;
  }
  const auto home_pose = arm.getCurrentPose("tool0").pose;
  const Eigen::Matrix3d place_rotation = Eigen::Quaterniond(
    home_pose.orientation.w, home_pose.orientation.x,
    home_pose.orientation.y, home_pose.orientation.z).normalized().toRotationMatrix();

  for (int round = 1; round <= std::max(1, max_rounds) && completed.size() < targets.size(); ++round) {
    RCLCPP_INFO(
      node->get_logger(), "pick round %d: %zu/%zu complete", round, completed.size(), targets.size());
    bool made_progress = false;
    for (const auto & target : targets) {
      if (completed.count(target) > 0) {
        continue;
      }
      const auto * spec = findSpec(target);
      if (!spec) {
        continue;
      }
      if (!moveToJoints(node, arm, observation_joints, target + " camera-clear observation")) {
        RCLCPP_WARN(node->get_logger(), "cannot reach observation posture for %s", target.c_str());
        continue;
      }
      writePickState(
        state_path, targets, placed, ++state_version, "perceiving", true, target);
      state_written_at = wallTimeSeconds();
      auto candidates = waitForFreshCandidates(
        node, csv_path, target, placed, state_written_at, perception_timeout);
      writePickState(
        state_path, targets, placed, ++state_version, "preflight", false, target);
      if (candidates.empty()) {
        continue;
      }
      // Grasp candidates describe the static object and remain valid after the
      // arm leaves the observation posture.  Start preflight from the same
      // per-object seed used by the upstream executor, with perception already
      // disabled so the moving arm cannot create false detections.
      if (!moveToJoints(node, arm, planningHomeForObject(target), target + " planning home")) {
        RCLCPP_WARN(node->get_logger(), "cannot reach planning home for %s", target.c_str());
        continue;
      }
      scene.removeCollisionObjects({target});
      GraspPlan selected;
      // Upstream behavior: candidate dry-runs are deliberately short, while
      // formal execution gets the full planning budget again.
      arm.setPlanningTime(candidate_planning_time);
      const bool plan_selected = selectPlan(
          node, arm, candidates, *spec, world_R_camera, world_t_camera,
          place_rotation, approach_distance, lift_distance, place_clearance,
          fallback_approach_distance, max_candidates, max_prefilter_candidates,
          max_tool_radius, ik_client, fast_ik_timeout, enable_fallback, selected);
      arm.setPlanningTime(execution_planning_time);
      if (!plan_selected) {
        RCLCPP_WARN(node->get_logger(), "no four-stage-valid candidate for %s", target.c_str());
        scene.applyCollisionObject(makeCollisionObject(*spec, spec->initial_center));
        continue;
      }
      const auto result = executeObject(
        node, arm, scene, attachment, gripper, *spec, selected, dry_run, tcp_pose_gate);
      if (result == AttemptResult::kPlaced) {
        placed.insert(target);
        completed.insert(target);
        made_progress = true;
        writePickState(state_path, targets, placed, ++state_version, "idle", false);
        state_written_at = wallTimeSeconds();
      } else if (result == AttemptResult::kPreflightOnly) {
        completed.insert(target);
        made_progress = true;
        scene.applyCollisionObject(makeCollisionObject(*spec, spec->initial_center));
      } else {
        attachment.setAttached(target, false);
        commandGripper(node, gripper, 0.10);
        scene.applyCollisionObject(makeCollisionObject(*spec, spec->initial_center));
      }
    }
    if (!made_progress) {
      RCLCPP_WARN(node->get_logger(), "round %d made no progress", round);
    }
  }

  writePickState(
    state_path, targets, placed, ++state_version,
    completed.size() == targets.size() ? "complete" : "incomplete", false);
  moveToJoints(node, arm, zero_joints, "final zero");
  const bool success = completed.size() == targets.size();
  RCLCPP_INFO(
    node->get_logger(), "%s: placed=%zu/%zu dry_run=%s",
    success ? "task complete" : "task incomplete", placed.size(), targets.size(),
    dry_run ? "true" : "false");
  shutdown();
  return success ? 0 : 1;
}

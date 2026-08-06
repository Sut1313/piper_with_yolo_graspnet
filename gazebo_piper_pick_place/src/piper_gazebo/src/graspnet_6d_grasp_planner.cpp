#include "piper_gazebo/grasp_planners.hpp"

#include <algorithm>
#include <cmath>

#include <rclcpp/rclcpp.hpp>

#include "piper_gazebo/motion_planning.hpp"
#include "piper_gazebo/pick_node.hpp"

// ============================================================================
// GraspNet 6D 候选规划器：从感知缓存中选出"完整可达"的抓取候选
//
//   流水线（4 段 AND，全部通过才选中）：
//     候选排序 → 视觉中心门控 → approach 可达预检 → grasp 可达预检
//     → open-gripper lift 可达预检 → place target 可达预检
//   任一环节失败即放弃该候选、继续遍历下一个
// ============================================================================
namespace piper_gazebo::pick_place {

/**
 * chooseReachableGraspCandidate() — 遍历 GraspNet 6D 候选，选出首个
 * 四段预检（approach→grasp→lift→place）全部通过的可达抓取方案。
 *
 *  候选处理顺序：
 *    1. 排序：视觉中心与 Gazebo 真实中心的距离升序、分数降序
 *       —— GraspNet 的 translation 是抓取中心而非物体中心，识别与
 *       attach 门控必须使用 CSV 的 object_center_* 字段
 *    2. round-robin 姿态桶重排：8 个姿态桶逐桶轮流取 1 个，
 *       保证不同接近方向的候选交替尝试（避免同姿态簇连续失败）
 *    3. 逐个候选执行 4 段 dry-run 预检（只规划不执行），全部通过即选中
 *
 *  @param node           PickNode（感知缓存读取 + 日志）
 *  @param move_group     MoveIt 规划接口
 *  @param object_template 物体模板（名称/放置点等）
 *  @param[out] selected_object     选中的抓取方案（TCP 位置已填充）
 *  @param[out] selected_orientation 选中的 TCP 姿态
 *  @return true=找到完整可达候选；false=无可达候选（上层可走位置回退）
 */
bool chooseReachableGraspCandidate(
    PickNode& node, moveit::planning_interface::MoveGroupInterface& move_group,
    const Object& object_template, double approach_clearance, double lift_z, double grasp_z_offset,
    double min_grasp_z, double tool_z_offset, double open_gripper_lift_height, double back_place_x,
    double back_place_y, const geometry_msgs::msg::Quaternion& back_place_orientation,
    Object& selected_object, geometry_msgs::msg::Quaternion& selected_orientation) {
  // 读取该物体的全部有效 6D 候选（已过滤无姿态/点数不足的目标）
  std::vector<PerceptionTarget> candidates;
  if (!node.readValidatedVisionObjectGraspCandidates(object_template.name, candidates)) {
    return false;
  }

  // 同时读取 Gazebo 真实位置：视觉中心门控以它为基准
  geometry_msgs::msg::Point actual_object;
  if (!node.readObjectPosition(object_template.name, actual_object)) {
    RCLCPP_ERROR(node.get_logger(),
                 "Cannot rank GraspNet candidates for %s: Gazebo pose is not readable.",
                 object_template.name.c_str());
    return false;
  }

  std::vector<size_t> order(candidates.size());
  for (size_t i = 0; i < order.size(); ++i) {
    order[i] = i;
  }
  // 第一步：按视觉中心距 Gazebo 中心距离升序 + 分数降序排列。
  // GraspNet 的 translation 是抓取中心，不是物体中心；用于识别和
  // attach 距离门控时必须使用 CSV 中的 object_center_* 字段。
  std::sort(order.begin(), order.end(), [&](size_t lhs, size_t rhs) {
    const auto dl = distanceToVerticalCylinder(candidates[lhs].object_center_world, actual_object);
    const auto dr = distanceToVerticalCylinder(candidates[rhs].object_center_world, actual_object);
    if (std::abs(dl.center - dr.center) > 1e-6) {
      return dl.center < dr.center;
    }
    return candidates[lhs].score > candidates[rhs].score;
  });

  // 第二步：按姿态簇做 round-robin 重排，保证每次尝试不同抓取方向。
  // 把四元数 x/y/z 三个分量的符号位编码成 8 个桶（0-7），同一桶 ≈ 同一
  // 抓取方向簇；重排后相邻尝试来自不同桶，避免"连续 N 个候选同方向
  // 失败"白白消耗预检时间（预检是耗时的 dry-run 规划）
  auto orientation_bucket = [](const geometry_msgs::msg::Quaternion& q) -> int {
    int b = 0;
    if (q.x >= 0)
      b |= 1;
    if (q.y >= 0)
      b |= 2;
    if (q.z >= 0)
      b |= 4;
    return b;
  };
  // 分配候选到桶（每个桶内保持距离排序）
  std::vector<std::vector<size_t>> buckets(8);
  for (size_t idx : order) {
    buckets[orientation_bucket(candidates[idx].orientation)].push_back(idx);
  }
  // round-robin 逐桶取一个
  std::vector<size_t> diverse_order;
  diverse_order.reserve(order.size());
  std::vector<size_t> bucket_ptr(8, 0);
  while (true) {
    bool any_left = false;
    for (int b = 0; b < 8; ++b) {
      if (bucket_ptr[b] < buckets[b].size()) {
        diverse_order.push_back(buckets[b][bucket_ptr[b]++]);
        any_left = true;
      }
    }
    if (!any_left)
      break;
  }
  order = std::move(diverse_order);

  const auto max_6d_candidates =
      static_cast<size_t>(std::max(1, node.get_parameter_or<int>("graspnet_6d_max_candidates", 1)));
  const auto max_6d_orientation_variants = static_cast<size_t>(
      std::max(1, node.get_parameter_or<int>("graspnet_6d_max_orientation_variants", 2)));
  const bool scan_only = node.get_parameter_or<bool>("graspnet_6d_scan_only", false);
  size_t tested_pose_count = 0;
  size_t approach_pass_count = 0;
  size_t grasp_pass_count = 0;
  size_t lift_pass_count = 0;
  size_t place_pass_count = 0;

  // 候选主遍历：按重排后的顺序逐候选做视觉中心门控 + 4 段预检
  for (size_t rank = 0; rank < order.size(); ++rank) {
    const size_t i = order[rank];
    const auto& candidate = candidates[i];
    // 视觉中心门控：GraspNet 的 translation 是抓取中心而非物体中心，
    // 故用 CSV 的 object_center 与 Gazebo 真实中心比对（阈值 0.08m）。
    // 超限说明该候选对应漂移/误检目标，直接拒绝、不进入 IK 预检
    const auto candidate_distance =
        distanceToVerticalCylinder(candidate.object_center_world, actual_object);
    if (candidate_distance.center > kVisionCandidateMaxCenterError) {
      RCLCPP_WARN(node.get_logger(),
                  "%s candidate %zu/%zu rejected before IK: visual object center "
                  "center_dist=%.4f m > %.4f m from Gazebo cylinder "
                  "(surface_dist=%.4f m).",
                  object_template.name.c_str(), i + 1, candidates.size(), candidate_distance.center,
                  kVisionCandidateMaxCenterError, candidate_distance.surface);
      continue;
    }

    // 用候选位置覆盖模板位置；z 取 max{候选 z + 抓取偏移 + TCP 偏移,
    // 最低抓取高度, 物理下限} —— GraspNet 的 z 是物体中心高度，
    // 须换算成 TCP 高度并防下探桌面
    Object object = object_template;
    object.x = candidate.world.x;
    object.y = candidate.world.y;
    object.z = std::max({candidate.world.z + grasp_z_offset + tool_z_offset, min_grasp_z,
                         kMinReachableTcpGraspZ});

    if (std::abs(object.z - candidate.world.z) > 1e-6) {
      RCLCPP_INFO(node.get_logger(), "%s candidate %zu GraspNet TCP z adjusted %.4f -> %.4f.",
                  object.name.c_str(), i + 1, candidate.world.z, object.z);
    }

    // 生成该候选的 TCP 姿态变体（当前仅 calib_18 一种），
    // 并按 graspnet_6d_max_orientation_variants 限制尝试数量
    auto tcp_orientations = graspNetTcpOrientationVariants(candidate.camera_rotation);
    if (tcp_orientations.size() > max_6d_orientation_variants) {
      tcp_orientations.resize(max_6d_orientation_variants);
    }

    // 姿态变体遍历：同一候选的不同 TCP 姿态逐个执行预检
    for (size_t orient_idx = 0; orient_idx < tcp_orientations.size(); ++orient_idx) {
      const auto& tcp_orientation = tcp_orientations[orient_idx].orientation;
      const auto poses = makeObjectPoses(object, tcp_orientation, approach_clearance, lift_z,
                                         grasp_z_offset, min_grasp_z, 0.0, tool_z_offset, false);
      RCLCPP_INFO(node.get_logger(),
                  "%s visual candidate %zu/%zu: center=(%.4f, %.4f, %.4f), "
                  "graspnet_grasp=(%.4f, %.4f, %.4f), score=%.4f, orient_variant=%zu, "
                  "orientation=%s, tcp_approach=(%.4f, %.4f, %.4f), "
                  "surface_dist=%.4f m, center_dist=%.4f m",
                  object.name.c_str(), i + 1, candidates.size(), candidate.object_center_world.x,
                  candidate.object_center_world.y, candidate.object_center_world.z,
                  candidate.world.x, candidate.world.y, candidate.world.z, candidate.score,
                  orient_idx, tcp_orientations[orient_idx].name.c_str(), poses.approach.position.x,
                  poses.approach.position.y, poses.approach.position.z, candidate_distance.surface,
                  candidate_distance.center);
      ++tested_pose_count;

      // ── 4 段预检流水线（AND 语义）：approach → grasp → lift → place ──
      // 每段都做"只规划不执行"的 dry-run，全过才认定该候选完整可达。
      // 提前在 attach 之前筛掉不可达段，避免执行到一半失败后被迫清理物体
      if (!canPlanFullPose(node, move_group, poses.approach,
                           object.name + " visual-center approach")) {
        continue;
      }
      ++approach_pass_count;

      if (!canPlanFullPose(node, move_group, poses.grasp, object.name + " visual-center grasp")) {
        continue;
      }
      ++grasp_pass_count;

      // 张开夹爪抬升段：attach 后需带着物体垂直抬起 open_gripper_lift_height
      auto open_lift_pose = poses.grasp;
      open_lift_pose.position.z += open_gripper_lift_height;
      if (!canPlanFullPose(node, move_group, open_lift_pose, object.name + " visual-center lift")) {
        continue;
      }
      ++lift_pass_count;

      // 放置段：以张开夹爪抬升位姿为起点平移到放置点（姿态沿用 home 姿态）
      auto back_place_pose = open_lift_pose;
      back_place_pose.position.x = back_place_x;
      back_place_pose.position.y = back_place_y;
      back_place_pose.orientation = back_place_orientation;
      if (canPlanFullPose(node, move_group, back_place_pose,
                          object.name + " visual-center place target")) {
        ++place_pass_count;
        // scan_only 模式：只统计可达性、不真正选中（用于抓取前扫描调试）
        if (scan_only) {
          RCLCPP_INFO(node.get_logger(),
                      "%s scan PASS candidate %zu/%zu orientation variant %zu "
                      "(%s); GraspNet TCP=(%.3f, %.3f, %.3f).",
                      object.name.c_str(), i + 1, candidates.size(), orient_idx,
                      tcp_orientations[orient_idx].name.c_str(), object.x, object.y, object.z);
          continue;
        }
        selected_object = object;
        selected_orientation = tcp_orientation;
        RCLCPP_INFO(node.get_logger(),
                    "%s selected visual center candidate %zu/%zu with orientation "
                    "variant %zu (%s); GraspNet TCP=(%.3f, %.3f, %.3f), "
                    "place TCP target=(%.3f, %.3f, %.3f) is reachable.",
                    object.name.c_str(), i + 1, candidates.size(), orient_idx,
                    tcp_orientations[orient_idx].name.c_str(), object.x, object.y, object.z,
                    back_place_pose.position.x, back_place_pose.position.y,
                    back_place_pose.position.z);
        return true;
      }
    }

    // 达到候选遍历上限（graspnet_6d_max_candidates，默认 1，上限 300）
    // 即停止：预检是耗时 dry-run 规划，不能无界遍历
    if (rank + 1 >= max_6d_candidates) {
      RCLCPP_WARN(node.get_logger(),
                  "%s tried %zu GraspNet 6D candidates without a complete "
                  "reachable plan.",
                  object_template.name.c_str(), max_6d_candidates);
      break;
    }
  }

  if (scan_only) {
    RCLCPP_INFO(node.get_logger(),
                "%s GraspNet 6D scan summary: tested=%zu, approach_pass=%zu, "
                "grasp_pass=%zu, lift_pass=%zu, place_pass=%zu.",
                object_template.name.c_str(), tested_pose_count, approach_pass_count,
                grasp_pass_count, lift_pass_count, place_pass_count);
  }

  RCLCPP_ERROR(node.get_logger(),
               "No complete GraspNet 6D candidate for %s has a reachable "
               "approach, grasp, open-gripper lift, and place transfer target. "
               "Position-only fallback may be used by the executor.",
               object_template.name.c_str());
  return false;
}

} // namespace piper_gazebo::pick_place

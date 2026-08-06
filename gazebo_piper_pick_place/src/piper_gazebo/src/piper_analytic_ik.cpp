#include "piper_gazebo/piper_analytic_ik.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>

#include <Eigen/Geometry>

// ============================================================================
// 解析 IK 求解器封装：调用 ssik Python 脚本（piper_ssik_ik.py）求 link6 逆解
//
//   变换链：TCP 目标位姿 → link6 目标位姿 → ssik 解析解 → 按种子代价排序
//   背景：MoveIt 数值 IK（KDL）对 GraspNet 全姿态目标求解率低、耗时长，
//         改用 ssik.prebuilt.agilex.piper_ik（非 Pieper 6R 专用解析求解器）。
//   解析解仍需要 MoveIt 做碰撞感知轨迹规划（本模块只替代 IK 采样阶段）
// ============================================================================
namespace piper_gazebo::pick_place {
namespace {

// TCP 在 link6 坐标系下的 Z 向偏移（米）：与 URDF 的
// gripper_base_to_grasp_tcp xyz="0 0 0.14" 同步。
// ssik 只解 base_link→link6，TCP 目标必须先折算回 link6 才能求解
constexpr double kTcpOffsetFromLink6Z = 0.14;

// Shell 单引号转义：把参数安全包进单引号（单引号本身转义为 '\''），
// 防止脚本路径 / 矩阵参数被 shell 解释（防注入 + 兼容空格路径）
std::string shellQuote(const std::string& value) {
  std::string quoted = "'";
  for (const char c : value) {
    if (c == '\'') {
      quoted += "'\\''";
    } else {
      quoted += c;
    }
  }
  quoted += "'";
  return quoted;
}

// 最短角度差：把 a-b 包裹到 [-π, π]。
// 用于关节角距离度量（避免 350° 与 10° 被误算成 340° 的距离）
double shortestAngleDistance(double a, double b) {
  double d = a - b;
  while (d > M_PI)
    d -= 2.0 * M_PI;
  while (d < -M_PI)
    d += 2.0 * M_PI;
  return d;
}

// 候选解与当前关节角的"种子代价"：各关节最短角度差平方和。
// 值越小说明该解离当前位姿越近、运动越省时（解排序的第一关键字）
double jointDistance(const std::vector<double>& candidate, const std::vector<double>& seed) {
  double sum = 0.0;
  const size_t n = std::min(candidate.size(), seed.size());
  for (size_t i = 0; i < n; ++i) {
    const double d = shortestAngleDistance(candidate[i], seed[i]);
    sum += d * d;
  }
  return sum;
}

// ROS 位姿 → Eigen 齐次变换（四元数归一化后构造）
Eigen::Isometry3d poseToEigen(const geometry_msgs::msg::Pose& pose) {
  Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y,
                       pose.orientation.z);
  q.normalize();
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = q.toRotationMatrix();
  transform.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  return transform;
}

// 定位 piper_ssik_ik.py：优先取源码目录（开发期），其次取 install 目录
// （已部署），最后退回仅依赖 PATH 中的脚本名
std::string defaultScriptPath() {
  const std::filesystem::path cwd = std::filesystem::current_path();
  const std::filesystem::path source_path =
      cwd / "src" / "piper_gazebo" / "scripts" / "piper_ssik_ik.py";
  if (std::filesystem::exists(source_path)) {
    return source_path.string();
  }

  const std::filesystem::path install_path =
      cwd / "install" / "piper_gazebo" / "lib" / "piper_gazebo" / "piper_ssik_ik.py";
  if (std::filesystem::exists(install_path)) {
    return install_path.string();
  }

  return "piper_ssik_ik.py";
}

} // namespace

/**
 * solvePiperAnalyticIkForTcp() — 求解 TCP 目标位姿的解析 IK 解列表
 *
 *  流程：
 *    ① TCP 位姿 → link6 位姿：右乘 TCP 偏移的逆变换（0.14m 沿 Z）。
 *       ssik 求解的是 base_link→link6，而本系统规划到 grasp_tcp
 *    ② 构造命令行调用 python3 piper_ssik_ik.py：link6 齐次矩阵与当前
 *       关节角种子以 17 位精度序列化（--matrix / --seed）
 *    ③ 逐行解析子进程 stdout：6 关节角 + residual（重投影残差）
 *    ④ 按 seed_cost（离当前关节角的距离）升序排序，同价时按 residual
 *       升序 —— 优先尝试"离当前位姿最近且数值最准"的解
 *
 *  返回空列表 = 该位姿解析意义上不可达（上游据此跳过候选，不做数值
 *  IK 兜底；解析空洞即为最终失败，由候选遍历提供回退）
 *
 *  @param target_tcp_pose 目标 TCP 位姿（world 系）
 *  @param current_joints  当前 6 关节角（种子）
 *  @param max_solutions   最多返回的解数（>=1）
 *  @param script_path     ssik 脚本路径，空则自动探测默认路径
 *  @return 已排序的解列表（可能为空）
 */
std::vector<PiperAnalyticIkSolution>
solvePiperAnalyticIkForTcp(const geometry_msgs::msg::Pose& target_tcp_pose,
                           const std::vector<double>& current_joints, int max_solutions,
                           const std::string& script_path) {
  if (current_joints.size() != 6) {
    return {};
  }

  const Eigen::Isometry3d target_tcp = poseToEigen(target_tcp_pose);
  Eigen::Isometry3d tcp_from_link6 = Eigen::Isometry3d::Identity();
  tcp_from_link6.translation() = Eigen::Vector3d(0.0, 0.0, kTcpOffsetFromLink6Z);
  const Eigen::Isometry3d target_link6 = target_tcp * tcp_from_link6.inverse();

  std::ostringstream command;
  command << "python3 " << shellQuote(script_path.empty() ? defaultScriptPath() : script_path)
          << " --max-solutions " << std::max(1, max_solutions) << " --matrix=";
  const Eigen::Matrix4d matrix = target_link6.matrix();
  std::ostringstream matrix_arg;
  matrix_arg << std::setprecision(17);
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      if (r != 0 || c != 0) {
        matrix_arg << ',';
      }
      matrix_arg << matrix(r, c);
    }
  }
  command << shellQuote(matrix_arg.str()) << " --seed=";
  std::ostringstream seed_arg;
  seed_arg << std::setprecision(17);
  for (const double joint : current_joints) {
    if (seed_arg.tellp() > 0) {
      seed_arg << ',';
    }
    seed_arg << joint;
  }
  command << shellQuote(seed_arg.str());

  std::unique_ptr<FILE, int (*)(FILE*)> pipe(popen(command.str().c_str(), "r"), pclose);
  if (!pipe) {
    throw std::runtime_error("Failed to invoke Piper ssik IK helper.");
  }

  // 解析子进程 stdout：每行 = 6 个关节角 + residual（注释行/空行直接跳过）
  std::vector<PiperAnalyticIkSolution> solutions;
  std::array<char, 512> buffer{};
  while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe.get()) != nullptr) {
    std::istringstream line(buffer.data());
    PiperAnalyticIkSolution solution;
    solution.joints.resize(6);
    if (!(line >> solution.joints[0] >> solution.joints[1] >> solution.joints[2] >>
          solution.joints[3] >> solution.joints[4] >> solution.joints[5] >> solution.residual)) {
      continue;
    }
    solution.seed_cost = jointDistance(solution.joints, current_joints);
    solutions.push_back(std::move(solution));
  }

  std::sort(solutions.begin(), solutions.end(),
            [](const PiperAnalyticIkSolution& a, const PiperAnalyticIkSolution& b) {
              if (std::abs(a.seed_cost - b.seed_cost) > 1e-9) {
                return a.seed_cost < b.seed_cost;
              }
              return a.residual < b.residual;
            });
  return solutions;
}

} // namespace piper_gazebo::pick_place

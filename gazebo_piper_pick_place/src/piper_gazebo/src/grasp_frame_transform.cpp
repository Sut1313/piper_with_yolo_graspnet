#include "piper_gazebo/grasp_common.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

// ============================================================================
// 坐标系变换工具实现：GraspNet 相机坐标系 → world 世界坐标系 → Piper TCP 坐标系
//
//   位置变换：cameraToWorld()            （kCameraToWorldPositionR + T）
//   姿态变换：cameraGraspRotationToWorldTcp()（kCameraToWorldRotationR × calib_18）
//   位姿序列：makeObjectPoses()          （approach→grasp→lift→transfer→release→retract）
//
//   物体统一按竖直圆柱近似（半径 0.024m、半长 0.03375m，与 grasp_common.hpp 一致）
// ============================================================================
namespace piper_gazebo::pick_place {

// 三个目标物体：名称 / 形状 / 当前坐标（运行时由感知与 Gazebo 刷新） / 放置点。
// 注意感知 key 映射：blue_sphere 在感知 CSV 中记录为 blue_cylinder
// （见 perceptionKeyForObject），其余物体名称一一对应
std::vector<Object> objects() {
  return {
      {"green_cylinder", "cylinder", 0.0, 0.0, 0.0, 0.300, -0.350},
      {"red_box", "cylinder", 0.0, 0.0, 0.0, 0.300, -0.350},
      {"blue_sphere", "cylinder", 0.0, 0.0, 0.0, 0.300, -0.350},
  };
}

// 计算点与"竖直圆柱"（物体近似体）的距离：
//   center : 两点欧氏距离 —— 门控判定的主基准（与 Gazebo 中心对齐的粗检）
//   surface: 扣除圆柱半径/半长后的表面距离 —— 更贴近物理接触的距离判断
DistanceToCylinder distanceToVerticalCylinder(const geometry_msgs::msg::Point& point,
                                              const geometry_msgs::msg::Point& center) {
  constexpr double kCylinderRadius = 0.024;
  constexpr double kCylinderHalfLength = 0.03375;
  const double dx = point.x - center.x;
  const double dy = point.y - center.y;
  const double dz = point.z - center.z;
  const double radial = std::sqrt(dx * dx + dy * dy);
  const double radial_gap = std::max(0.0, radial - kCylinderRadius);
  const double axial_gap = std::max(0.0, std::abs(dz) - kCylinderHalfLength);
  // center=纯欧氏距离；surface=径向间隙与轴向间隙的合成（已扣除圆柱尺寸）
  return {std::sqrt(dx * dx + dy * dy + dz * dz),
          std::sqrt(radial_gap * radial_gap + axial_gap * axial_gap)};
}

// 用四元数 q 旋转向量 v（Rodrigues 形式：v + 2*s*(u×v) + 2*(u×(u×v))）
Vec3 rotate(const geometry_msgs::msg::Quaternion& q, const Vec3& v) {
  const Vec3 u{q.x, q.y, q.z};
  const double s = q.w;
  const Vec3 uv{
      u.y * v.z - u.z * v.y,
      u.z * v.x - u.x * v.z,
      u.x * v.y - u.y * v.x,
  };
  const Vec3 uuv{
      u.y * uv.z - u.z * uv.y,
      u.z * uv.x - u.x * uv.z,
      u.x * uv.y - u.y * uv.x,
  };
  return {
      v.x + 2.0 * (s * uv.x + uuv.x),
      v.y + 2.0 * (s * uv.y + uuv.y),
      v.z + 2.0 * (s * uv.z + uuv.z),
  };
}

// 相机系点坐标 → 世界系：world = kCameraToWorldPositionR * camera + kCameraToWorldT。
// 位置变换必须用 PositionR（含坐标轴手性翻转），
// 不能与姿态矩阵 RotationR 混用（两者互为镜像，中间行符号相反）
geometry_msgs::msg::Point cameraToWorld(const Vec3& camera) {
  geometry_msgs::msg::Point world;
  // 行主序矩阵乘：world = R * [x y z]^T + T（展开为三个分量赋值）
  world.x = kCameraToWorldPositionR[0][0] * camera.x + kCameraToWorldPositionR[0][1] * camera.y +
            kCameraToWorldPositionR[0][2] * camera.z + kCameraToWorldT[0];
  world.y = kCameraToWorldPositionR[1][0] * camera.x + kCameraToWorldPositionR[1][1] * camera.y +
            kCameraToWorldPositionR[1][2] * camera.z + kCameraToWorldT[1];
  world.z = kCameraToWorldPositionR[2][0] * camera.x + kCameraToWorldPositionR[2][1] * camera.y +
            kCameraToWorldPositionR[2][2] * camera.z + kCameraToWorldT[2];
  return world;
}

// 3x3 旋转矩阵乘法（三种参数形式：C 数组×array / array×C 数组 / array×array），
// 结果均为 9 元素行主序数组，构成"相机→世界→TCP"的链式姿态变换
std::array<double, 9> multiplyRotation(const double a[3][3], const std::array<double, 9>& b) {
  std::array<double, 9> out{};
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      out[r * 3 + c] = a[r][0] * b[c] + a[r][1] * b[3 + c] + a[r][2] * b[6 + c];
    }
  }
  return out;
}

std::array<double, 9> multiplyRotation(const std::array<double, 9>& a, const double b[3][3]) {
  std::array<double, 9> out{};
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      out[r * 3 + c] = a[r * 3] * b[0][c] + a[r * 3 + 1] * b[1][c] + a[r * 3 + 2] * b[2][c];
    }
  }
  return out;
}

std::array<double, 9> multiplyRotation(const std::array<double, 9>& a,
                                       const std::array<double, 9>& b) {
  std::array<double, 9> out{};
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      out[r * 3 + c] = a[r * 3] * b[c] + a[r * 3 + 1] * b[3 + c] + a[r * 3 + 2] * b[6 + c];
    }
  }
  return out;
}

// 旋转矩阵 → 四元数（Shepperd 方法：按迹与对角元分支选取数值最稳定的公式，
// 最后归一化；对 det=-1 的镜像矩阵也适用——翻转信息被归一化吸收）
geometry_msgs::msg::Quaternion quaternionFromRotation(const std::array<double, 9>& m) {
  geometry_msgs::msg::Quaternion q;
  const double trace = m[0] + m[4] + m[8];
  if (trace > 0.0) {
    const double s = std::sqrt(trace + 1.0) * 2.0;
    q.w = 0.25 * s;
    q.x = (m[7] - m[5]) / s;
    q.y = (m[2] - m[6]) / s;
    q.z = (m[3] - m[1]) / s;
  } else if (m[0] > m[4] && m[0] > m[8]) {
    const double s = std::sqrt(1.0 + m[0] - m[4] - m[8]) * 2.0;
    q.w = (m[7] - m[5]) / s;
    q.x = 0.25 * s;
    q.y = (m[1] + m[3]) / s;
    q.z = (m[2] + m[6]) / s;
  } else if (m[4] > m[8]) {
    const double s = std::sqrt(1.0 + m[4] - m[0] - m[8]) * 2.0;
    q.w = (m[2] - m[6]) / s;
    q.x = (m[1] + m[3]) / s;
    q.y = 0.25 * s;
    q.z = (m[5] + m[7]) / s;
  } else {
    const double s = std::sqrt(1.0 + m[8] - m[0] - m[4]) * 2.0;
    q.w = (m[3] - m[1]) / s;
    q.x = (m[2] + m[6]) / s;
    q.y = (m[5] + m[7]) / s;
    q.z = 0.25 * s;
  }

  // 归一化：Shepperd 各分支产出的四元数可能未归一（镜像矩阵时尤甚）
  const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (norm > 1e-9) {
    q.x /= norm;
    q.y /= norm;
    q.z /= norm;
    q.w /= norm;
  }
  return q;
}

// 相机系抓取旋转 → 世界系 TCP 旋转，完整变换链：
//   world_tcp = kCameraToWorldRotationR * camera_grasp_rotation * kGraspNetToPiperTcpR
//   ① 相机→世界：左乘姿态外参矩阵（注意姿态必须用 RotationR 而非 PositionR）
//   ② 抓取帧→TCP：右乘 calib_18 映射（GraspNet 接近轴 X → TCP 前向 Z）
geometry_msgs::msg::Quaternion
cameraGraspRotationToWorldTcp(const std::array<double, 9>& camera_grasp_rotation) {
  const auto world_grasp = multiplyRotation(kCameraToWorldRotationR, camera_grasp_rotation);
  const auto world_tcp = multiplyRotation(world_grasp, kGraspNetToPiperTcpR);
  return quaternionFromRotation(world_tcp);
}

// 与上一个重载相同，但 grasp→tcp 映射矩阵由调用方显式指定（多映射枚举调试用）
geometry_msgs::msg::Quaternion
cameraGraspRotationToWorldTcp(const std::array<double, 9>& camera_grasp_rotation,
                              const std::array<double, 9>& grasp_to_tcp_rotation) {
  const auto world_grasp = multiplyRotation(kCameraToWorldRotationR, camera_grasp_rotation);
  const auto world_tcp = multiplyRotation(world_grasp, grasp_to_tcp_rotation);
  return quaternionFromRotation(world_tcp);
}

// 生成抓取候选的全部 TCP 姿态变体。当前仅保留 calib_18 一种（单选），
// 返回值恒为一个元素的列表；返回空时上层直接跳过该候选
std::vector<OrientationVariant>
graspNetTcpOrientationVariants(const std::array<double, 9>& camera_grasp_rotation) {
  std::vector<std::pair<std::string, std::array<double, 9>>> mappings;

  // 仅使用标定映射 calib_18（GraspNet 接近轴 X → TCP 前向 Z）。
  // 历史枚举变体（6 置换 × 8 符号）会把接近轴映射到 TCP 的横向轴，
  // 导致筛选出的姿态"可达但背对物体"（实测接近角 127.7° 失败案例）。
  // IK 不可达时由上层跳过该候选、继续遍历下一候选（graspnet_6d_max_candidates）。
  std::array<double, 9> preferred{
      0.0, -1.0, 0.0, 0.0, 0.0, -1.0, 1.0, 0.0, 0.0,
  };
  mappings.push_back({"graspnet_tcp_calib_18_preferred", preferred});

  std::vector<OrientationVariant> variants;
  variants.reserve(mappings.size());
  for (const auto& mapping : mappings) {
    variants.push_back(
        {mapping.first, cameraGraspRotationToWorldTcp(camera_grasp_rotation, mapping.second)});
  }
  return variants;
}

// 垂直向下（Z 轴朝下）的 TCP 姿态，供仅位置的回退抓取路径使用：
// 不依赖 GraspNet 的方向预测，只保证末端从正上方竖直下压
geometry_msgs::msg::Quaternion topDownTcpOrientation() {
  std::array<double, 9> m{
      1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0,
  };
  return quaternionFromRotation(m);
}

// 物体名 → 感知 CSV / 缓存目标名映射：GraspNet 感知侧把蓝色球体
// 识别并记录为 blue_cylinder（训练类别与场景一致），其余物体一一对应
std::string perceptionKeyForObject(const std::string& object_name) {
  if (object_name == "blue_sphere") {
    return "blue_cylinder";
  }
  return object_name;
}

// 由坐标与姿态组装 TCP 位姿（工具函数，供 makeObjectPoses 复用）
geometry_msgs::msg::Pose tcpPose(double tool_x, double tool_y, double tool_z,
                                 const geometry_msgs::msg::Quaternion& orientation) {
  geometry_msgs::msg::Pose target;
  target.position.x = tool_x;
  target.position.y = tool_y;
  target.position.z = tool_z;
  target.orientation = orientation;
  return target;
}

// 在给定 TCP 位姿上叠加世界系偏移（用于从抓取点生成抬升/接近位姿）
geometry_msgs::msg::Pose offsetTcpPose(const geometry_msgs::msg::Pose& grasp_pose,
                                       const Vec3& world_offset) {
  geometry_msgs::msg::Pose target = grasp_pose;
  target.position.x += world_offset.x;
  target.position.y += world_offset.y;
  target.position.z += world_offset.z;
  return target;
}

// 由物体信息与抓取姿态生成完整 6 段位姿序列：
//   grasp_tcp_z 取 max{物体高度+抓取偏移+TCP 偏移, 最低抓取高度, 物理下限} ——
//   GraspNet 的 z 是物体中心高度，须换算成 TCP 高度并做下限保护（防下探桌面）
//   lift 为抓取点垂直抬升 lift_z；release/retract 对最后一个物体再抬高
//   kLastObjectHeightOffset，为叠放留出避碰空间
ObjectPoses makeObjectPoses(const Object& object, const geometry_msgs::msg::Quaternion& orientation,
                            double approach_clearance, double lift_z, double grasp_z_offset,
                            double min_grasp_z, double release_z, double tool_z_offset,
                            bool is_last) {
  const double release_target_z = release_z + (is_last ? kLastObjectHeightOffset : 0.0);
  const double retract_target_z = lift_z + (is_last ? kLastObjectHeightOffset : 0.0);
  const double grasp_tcp_z = std::max({object.z + grasp_z_offset + tool_z_offset, min_grasp_z,
                                       kMinReachableTcpGraspZ});

  const auto grasp_pose = tcpPose(object.x, object.y, grasp_tcp_z, orientation);
  const auto approach_pose = tcpPose(object.x, object.y, grasp_tcp_z + approach_clearance,
                                     orientation);
  // 抬升量 = 目标抬升高度 - 抓取点自身 z（max 防抬升量为负，即不允许下压）
  const Vec3 lift_offset{0.0, 0.0, std::max(0.0, lift_z - grasp_tcp_z)};

  // 位姿序列（6 段）：transfer 高度 = 物体高度 + 抬升量（带物转移不蹭地面）；
  // release/retract 高度 = release_target_z / retract_target_z（含叠放偏移）
  return {
      approach_pose,  // 1. 接近点：抓取点上方 approach_clearance 处
      grasp_pose,     // 2. 抓取点：TCP 接触物体的位姿
      offsetTcpPose(grasp_pose, lift_offset),                             // 3. 抬升点
      tcpPose(object.place_x, object.place_y, object.z + lift_offset.z, orientation),  // 4. 转移点
      tcpPose(object.place_x, object.place_y, release_target_z, orientation),          // 5. 释放点
      tcpPose(object.place_x, object.place_y, retract_target_z, orientation),          // 6. 缩回点
  };
}

} // namespace piper_gazebo::pick_place

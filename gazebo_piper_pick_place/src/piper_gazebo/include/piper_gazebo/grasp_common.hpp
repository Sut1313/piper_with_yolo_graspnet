#pragma once

#include <array>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

// ============================================================================
// 本头文件：抓取任务的全部共享常量、数据结构与坐标系变换/位姿生成工具声明。
// 常量按"标识 / 规划 / 相机外参 / 门控阈值 / 物体几何 / 坐标映射"分组，
// 阈值含义与取值依据见各组注释。
// ============================================================================
namespace piper_gazebo::pick_place {

// ── 基础标识：节点名 / MoveIt 规划组名 / 末端执行器 link ──
constexpr const char* kNodeName = "piper_gazebo_pick_and_place";
constexpr const char* kMoveGroupName = "arm";
constexpr const char* kEndEffectorLink = "grasp_tcp";

// ── 笛卡尔直线规划与码放常量 ──
// kCartesianStep       : 笛卡尔路径插值步长（米），越小路径越精确但越耗时
// kCartesianMinFraction: 直线规划最低覆盖率，低于此值判定失败并降级为关节空间规划
// kLastObjectHeightOffset: 最后一个物体码放时的额外抬升高度（叠放避碰）
constexpr double kCartesianStep = 0.005;
constexpr double kCartesianMinFraction = 0.9;
constexpr double kLastObjectHeightOffset = 0.04;

// ── 相机外参（GraspNet 相机坐标系 → world 世界坐标系），三点反求标定得出 ──
// kCameraToWorldPositionR: 位置变换矩阵 —— 用于点坐标（cameraToWorld()），
//                          行序含手性翻转以对齐相机系与 ROS 世界系
// kCameraToWorldRotationR: 姿态变换矩阵 —— 用于方向/四元数变换
//                          （cameraGraspRotationToWorldTcp()），与位置矩阵互为
//                          镜像（中间行符号相反），两者不可混用
// kCameraToWorldT        : 相机光心在世界系的平移（米），
//                          标定残差 <6cm，有效俯仰角约 55.9°
constexpr double kCameraToWorldPositionR[3][3] = {
    {0.0, -0.82806033, 0.56063899},
    {-1.0, 0.0, 0.0},
    {0.0, -0.56063899, -0.82806033},
};
constexpr double kCameraToWorldRotationR[3][3] = {
    {0.0, -0.82806033, 0.56063899},
    {1.0, 0.0, 0.0},
    {0.0, -0.56063899, -0.82806033},
};
constexpr double kCameraToWorldT[3] = {0.072, 0.021, 0.548};

// ── 感知数据与门控阈值 ──
// kPerceptionMaxAgeSec          : 感知数据最大有效年龄（秒），过期即要求刷新
// kMinPerceptionPoints          : 目标点云最少点数，不足视为无效目标
// kVisionTargetMaxError         : 视觉目标与 Gazebo 真实位置最大允许偏差，
//                                 超限拒绝（防感知漂移污染抓取坐标）
// kVisionCandidateMaxCenterError: 候选视觉中心与 Gazebo 物体中心的距离门控，
//                                 超限直接拒绝该候选（不进入 IK 预检）
// kMinReachableTcpGraspZ        : TCP 抓取高度物理下限（防规划出低于桌面的点）
// kDefaultMaxGraspDistance      : 默认最大抓取距离（TCP 与物体距离门控）
// kMaxAttachCenterDistance      : attach 前 TCP 中心与物体中心的最大距离门控
constexpr double kPerceptionMaxAgeSec = 20.0;
constexpr int kMinPerceptionPoints = 80;
constexpr double kVisionTargetMaxError = 0.08;
constexpr double kVisionCandidateMaxCenterError = 0.08;
constexpr double kMinReachableTcpGraspZ = 0.005;
constexpr double kDefaultMaxGraspDistance = 0.08;
constexpr double kMaxAttachCenterDistance = 0.08;

// 抓取物体几何（与 piper_pick.world 及 grasp_frame_transform.cpp 一致）
constexpr double kObjectCylinderRadius = 0.024;
constexpr double kObjectCylinderLength = 0.0675;
// 规划时间预算：候选预检（dry-run）用更短时间，正式执行用更长时间
constexpr double kCandidatePlanningTime = 5.0;
constexpr double kExecutionPlanningTime = 10.0;

// ── GraspNet 抓取坐标系 → Piper TCP 坐标系的旋转映射（calib_18 唯一方案）──
// GraspNet 抓取帧的接近轴为 X，Piper TCP 的前向轴为 Z，
// 映射为：X→Z、Y→-X、Z→-Y（实测接近角 127.7° 失败案例后收敛到该方案，
// 历史枚举变体已删除）
constexpr double kGraspNetToPiperTcpR[3][3] = {
    {0.0, -1.0, 0.0},
    {0.0, 0.0, -1.0},
    {1.0, 0.0, 0.0},
};

// 目标物体描述：名称 / 形状 / 实时世界坐标 / 放置点坐标
struct Object
{
  std::string name;
  std::string shape;
  double x;
  double y;
  double z;
  double place_x;
  double place_y;
};

struct Vec3
{
  double x;
  double y;
  double z;
};

// 一次抓取动作的完整 TCP 位姿序列（执行流程中的 6 个关键位姿）
struct ObjectPoses
{
  geometry_msgs::msg::Pose approach; // 接近点：抓取点上方 approach_clearance 处
  geometry_msgs::msg::Pose grasp;    // 抓取点：与物体接触的 TCP 位姿
  geometry_msgs::msg::Pose lift;     // 抬升点：抓取点垂直抬高后的位姿
  geometry_msgs::msg::Pose transfer; // 转移点：移动到放置点上方
  geometry_msgs::msg::Pose release;  // 释放点：放置点（最后一个物体有额外抬升）
  geometry_msgs::msg::Pose retract;  // 缩回点：释放点再抬高后的离开位姿
};

// GraspNet 感知目标：一条来自 perception_grasps.csv 的候选抓取记录。
// 同一物体可有多条候选，按视觉中心门控与分数排序筛选。
// 注意：GraspNet 的 translation 是"抓取中心"而非物体中心，
// 识别与距离门控必须用 object_center_* 字段
struct PerceptionTarget
{
  Vec3 camera;               // 抓取中心在相机坐标系下的坐标
  Vec3 object_center_camera; // 物体中心在相机坐标系下的坐标（识别/门控用）
  geometry_msgs::msg::Point world;               // 抓取中心转换到世界系（GraspNet TCP 位置）
  geometry_msgs::msg::Point object_center_world; // 物体中心转换到世界系（门控基准）
  geometry_msgs::msg::Quaternion orientation;    // 世界系抓取姿态（四元数）
  std::array<double, 9> camera_rotation{};       // 相机系抓取旋转矩阵（3x3 行主序）
  double score{0.0};   // GraspNet 置信度分数（候选排序的次关键字）
  double width{0.0};   // 抓取宽度（米）
  double height{0.0};  // 抓取高度（米）
  double depth{0.0};   // 抓取深度（米）
  double link6_grasp_z{0.0}; // link6 系下的抓取高度（调试用）
  double tcp_forward_z{0.0}; // TCP 前向轴在世界 Z 方向的分量（接近方向判据）
  int count{0};               // 该目标对应的点云点数
  bool has_grasp_pose{false}; // 是否含完整 6D 抓取姿态（仅有位置不算）
};

// 点到"竖直圆柱"的距离结果：center=两点欧氏距离, surface=到圆柱表面的距离
// （surface 已扣除半径/半长，用于门控时更贴近物理接触判断）
struct DistanceToCylinder
{
  double center{0.0};
  double surface{0.0};
};

// 一种 GraspNet→TCP 坐标映射方案（当前仅 calib_18）及其产出的世界系姿态
struct OrientationVariant
{
  std::string name;                         // 映射方案名（日志标识）
  geometry_msgs::msg::Quaternion orientation; // 该方案下的世界系 TCP 姿态
};

// 目标物体列表（x/y/z 为运行时占位，由 Gazebo/感知实时刷新；place_x/y 为放置点）
std::vector<Object> objects();
// 计算点与竖直圆柱（物体近似体）的距离：center=中心距, surface=表面距
DistanceToCylinder distanceToVerticalCylinder(const geometry_msgs::msg::Point& point,
                                              const geometry_msgs::msg::Point& center);
// 用四元数 q 旋转向量 v
Vec3 rotate(const geometry_msgs::msg::Quaternion& q, const Vec3& v);
// 相机系点坐标 → 世界系（位置矩阵 kCameraToWorldPositionR + 平移 kCameraToWorldT）
geometry_msgs::msg::Point cameraToWorld(const Vec3& camera);
// 相机系抓取旋转 → 世界系 TCP 旋转（默认 calib_18 映射，变换链见实现文件）
geometry_msgs::msg::Quaternion
cameraGraspRotationToWorldTcp(const std::array<double, 9>& camera_grasp_rotation);
// 同上，但 grasp→tcp 映射矩阵由调用方显式指定（多映射枚举调试用）
geometry_msgs::msg::Quaternion
cameraGraspRotationToWorldTcp(const std::array<double, 9>& camera_grasp_rotation,
                              const std::array<double, 9>& grasp_to_tcp_rotation);
// 生成抓取候选的全部 TCP 姿态变体（当前仅 calib_18 一种，单选原因见实现）
std::vector<OrientationVariant>
graspNetTcpOrientationVariants(const std::array<double, 9>& camera_grasp_rotation);
// 垂直向下（俯视）的 TCP 姿态，用于仅位置的回退抓取路径
geometry_msgs::msg::Quaternion topDownTcpOrientation();
// 物体名 → 感知 CSV/缓存目标名映射（blue_sphere 感知侧记录为 blue_cylinder）
std::string perceptionKeyForObject(const std::string& object_name);
// 由坐标与姿态组装一个 TCP 位姿
geometry_msgs::msg::Pose tcpPose(double tool_x, double tool_y, double tool_z,
                                 const geometry_msgs::msg::Quaternion& orientation);
// 在给定 TCP 位姿上叠加世界系偏移量
geometry_msgs::msg::Pose offsetTcpPose(const geometry_msgs::msg::Pose& grasp_pose,
                                       const Vec3& world_offset);
// 由物体信息 + 抓取姿态生成完整 6 段位姿序列
// （approach→grasp→lift→transfer→release→retract，z 含 TCP 偏移与下限保护）
ObjectPoses makeObjectPoses(const Object& object, const geometry_msgs::msg::Quaternion& orientation,
                            double approach_clearance, double lift_z, double grasp_z_offset,
                            double min_grasp_z, double release_z, double tool_z_offset, bool is_last);

} // namespace piper_gazebo::pick_place

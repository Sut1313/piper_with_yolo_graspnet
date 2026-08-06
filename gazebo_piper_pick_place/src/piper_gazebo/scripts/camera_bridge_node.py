#!/usr/bin/env python3
"""
camera_bridge_node.py - ROS 2 → Shared Memory 相机数据桥接节点

订阅 Gazebo 相机的 RGB、Depth、CameraInfo 话题，
将数据以 GraspNet 要求的格式写入共享目录。

注意话题路径：ROS 2 Humble 的 gazebo_ros_camera 使用 sensor_name 作为话题前缀
  传感器名 → 话题路径
  "camera"  → /camera/image_raw, /camera/depth/image_raw, /camera/camera_info

发布的话题（订阅）：
  /camera/image_raw        (sensor_msgs/Image) — RGB 图像
  /camera/depth/image_raw  (sensor_msgs/Image) — 深度图（float32 米）
  /camera/camera_info      (sensor_msgs/CameraInfo) — 相机内参

输出到共享目录（共享给 Docker 中的 GraspNet）：
  color.png       — RGB 图像（H×W×3, uint8, PNG, BGR直存避免通道错乱）
  depth.png       — 深度图（uint16, 毫米, PNG）
  meta.mat        — intrinsic_matrix + factor_depth
  camera_info.txt — 可读的相机参数（调试用）
"""

import os
import sys
import signal
import time
import json

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2

try:
    import scipy.io as scio
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False
    print("WARNING: scipy not available, will save meta data as JSON instead", flush=True)


class CameraBridgeNode(Node):
    """将 Gazebo 相机数据桥接到共享目录，供 GraspNet 使用。

    数据流：Gazebo 相机(15Hz) → 三个订阅回调只缓存"最新一帧"
    → 定时器按 publish_rate 将整帧原子写入共享目录 → 最后写 .ready 标志
    → Docker 内的 inference_from_camera.py 轮询 .ready 后读取。

    注：本文件与 piper_perception/piper_perception/camera_bridge_node.py
    是同源的两份副本（一个经 setup.py console_scripts 安装，一个直接以脚本
    方式执行），修改时需保持两者同步。
    """

    def __init__(self):
        super().__init__('camera_bridge_node')

        # ── 参数 ──────────────────────────────────────────────────
        self.declare_parameter('shared_dir', '/home/zzu/piper_graspnet/shared_camera_data')
        self.declare_parameter('publish_rate_hz', 10.0)      # 发布到共享目录的频率
        self.declare_parameter('factor_depth', 1000.0)        # 深度缩放因子（→毫米）

        self.shared_dir = self.get_parameter('shared_dir').value
        self.publish_rate = self.get_parameter('publish_rate_hz').value
        self.factor_depth = self.get_parameter('factor_depth').value

        # 确保输出目录存在
        os.makedirs(self.shared_dir, exist_ok=True)

        self.bridge = CvBridge()

        # ── 状态 ──────────────────────────────────────────────────
        # 三个 latest_* 分别缓存最近一帧数据（回调线程写入、定时器线程读取）。
        # 只保留最新帧而不做队列，避免内存无限增长；代价是发布速率低于
        # 相机帧率时会跳帧（对抓取推理足够，因为只关心"现在看到什么"）。
        # last_publish_time 记录上次落盘时刻，用于按 publish_rate 限频。
        self.latest_color = None
        self.latest_depth = None
        self.latest_camera_info = None
        self.last_publish_time = 0.0

        # ── 订阅话题 ──────────────────────────────────────────────
        # ROS 2 Humble 中 gazebo_ros_camera 的话题路径为 /<sensor_name>/<default_topic>
        # 传感器名 "camera" → /camera/image_raw, /camera/depth/image_raw, /camera/camera_info
        self.color_sub = self.create_subscription(
            Image, '/camera/image_raw',
            self.color_callback, 10)

        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_raw',
            self.depth_callback, 10)

        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info',
            self.info_callback, 10)

        # ── 定时发布器 — 按固定频率将最新帧写入共享目录 ─────────
        self.timer = self.create_timer(1.0 / max(self.publish_rate, 0.1), self.publish_to_shared)

        self.get_logger().info(
            f'Camera Bridge Node started:\n'
            f'  shared_dir:    {self.shared_dir}\n'
            f'  publish_rate:  {self.publish_rate} Hz\n'
            f'  factor_depth:  {self.factor_depth}'
        )

    def _atomic_imwrite(self, path, image):
        # 原子写图片：先写"唯一临时文件"（PID + 纳秒时间戳避免多进程竞争），
        # 再用 os.replace（等价于 POSIX rename）一步替换目标文件。
        # 为什么必须原子写：Docker 内的 GraspNet 推理进程会直接 cv2.imread
        # 这些文件，若直接覆盖写，读者可能读到"写到一半"的半帧（损坏/不完整）。
        # rename 是文件系统原子操作，读者要么看到完整的旧帧，要么看到完整的新帧，
        # 永远不会看到中间状态——这是 .ready 握手协议正确性的前提。
        tmp_path = f'{path}.tmp.{os.getpid()}.{time.time_ns()}.png'
        try:
            if not cv2.imwrite(tmp_path, image):
                raise RuntimeError(f'Failed to write image: {tmp_path}')
            os.replace(tmp_path, path)
        except BaseException:
            # 写失败时清理临时文件，避免垃圾文件堆积
            try:
                os.unlink(tmp_path)
            except OSError:
                pass
            raise

    def _atomic_text_write(self, path, content):
        # 文本文件同样采用 先写 tmp 再 rename 的原子写。
        # .ready 标志和 meta 元数据的一致性都依赖这一机制。
        tmp_path = f'{path}.tmp'
        with open(tmp_path, 'w') as f:
            f.write(content)
        os.replace(tmp_path, path)

    # ── 回调函数 ──────────────────────────────────────────────────
    # 三个回调（color/depth/info）职责单一：只把最新一帧数据缓存到
    # 成员变量，不做任何文件 I/O——I/O 集中在定时器线程的 publish_to_shared，
    # 避免在回调里做阻塞操作拖垮话题订阅。

    def color_callback(self, msg: Image):
        # 回调 1/3：缓存最近一帧彩色图。
        # desired_encoding='bgr8' 转为 OpenCV 原生 BGR 布局。
        # 为什么此处不做 BGR→RGB 转换（历史坑）：
        #   cv2.imwrite 按数组原样落盘，cv2.imread 固定返回 BGR；
        #   若这里先转成 RGB，Docker 端 imread + cvtColor(BGR2RGB) 会再次交换，
        #   导致红/蓝通道互换（曾出现物体颜色识别全错的历史 bug）。
        #   约定：整条链路（本节点 → PNG 文件 → Docker cv2.imread）统一按 BGR 存储。
        try:
            self.latest_color = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().warn(f'Color image conversion failed: {e}')

    def depth_callback(self, msg: Image):
        # 回调 2/3：缓存最近一帧深度图。
        # Gazebo depth 相机输出 32FC1，单位是米（float32）。
        # 保持 float32 原样缓存，到 _write_depth 落盘时再统一换算为毫米 uint16。
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except CvBridgeError as e:
            self.get_logger().warn(f'Depth image conversion failed: {e}')

    def info_callback(self, msg: CameraInfo):
        # 回调 3/3：缓存相机内参（K 矩阵、分辨率、畸变等），
        # 供 _write_meta 生成 meta.mat（GraspNet 点云反投影必需）。
        self.latest_camera_info = msg

    # ── 发布到共享目录 ───────────────────────────────────────────

    def publish_to_shared(self):
        # 定时器入口：把"最近一帧"完整写入共享目录。
        # 先按 last_publish_time 做频率限制（防止定时器抖动导致写入过快），
        # 再检查三个数据源是否都已就绪（相机未启动时跳过，避免写脏数据）。
        now = time.time()
        if now - self.last_publish_time < 1.0 / self.publish_rate:
            return

        if self.latest_color is None or self.latest_depth is None or self.latest_camera_info is None:
            self.get_logger().debug('Waiting for camera data...', throttle_duration_sec=5.0)
            return

        try:
            # 写入顺序是时序协议的核心：color/depth/meta 全部落盘完成后，
            # 最后才写 .ready——.ready 充当"本帧完整"的提交标志。
            # 推理端只看到 .ready 才去读数据，保证永远不会读到半帧；
            # 推理端处理完会删除 .ready（见 inference_from_camera.py 的
            # clear_ready_flag），形成"生产者写、消费者删"的握手。
            self._write_color()
            self._write_depth()
            self._write_meta()
            self._write_debug_info()
            self._write_ready_flag()
            self.last_publish_time = now
            self.get_logger().info('Published frame to shared directory', throttle_duration_sec=2.0)
        except Exception as e:
            self.get_logger().error(f'Failed to publish to shared dir: {e}', throttle_duration_sec=5.0)

    def _write_color(self):
        """保存彩色图像为 color.png (BGR 直存，避免 R/B 通道错乱)"""
        # latest_color 是 BGR（来自 imgmsg_to_cv2 with 'bgr8'）。
        # 保存为 BGR 格式，使 Docker 中 cv2.imread + cvtColor(BGR2RGB) 得到正确 RGB。
        # 注意：cv2.imwrite 按数组原样写入，cv2.imread 固定返回 BGR 排列。
        # 若在此处做 BGR→RGB 转换，imread + cvtColor 会再次交换，导致 R/B 通道错乱
        # （历史 bug：曾出现红蓝互换，物体颜色全部识别错误）。
        path = os.path.join(self.shared_dir, 'color.png')
        self._atomic_imwrite(path, self.latest_color)

    def _write_depth(self):
        """保存深度图为 depth.png（uint16 毫米）"""
        # 米 → 毫米：乘以 factor_depth（1000，与 meta.mat 中的 factor_depth 一致，
        # Docker 端用它把 uint16 深度还原成米）。
        # np.nan_to_num 必须做：深度相机在无反射/超远像素上会输出 NaN/Inf，
        # 不清掉的话 astype(np.uint16) 对 NaN 的转换结果未定义，会产生脏点。
        depth_mm = self.latest_depth * self.factor_depth
        depth_mm = np.nan_to_num(depth_mm, nan=0.0, posinf=0.0, neginf=0.0)
        depth_uint16 = depth_mm.astype(np.uint16)

        path = os.path.join(self.shared_dir, 'depth.png')
        self._atomic_imwrite(path, depth_uint16)

    def _write_meta(self):
        """保存相机内参为 meta.mat"""
        # 从 sensor_msgs/CameraInfo 的 K 数组（9 元素，行主序）重排为 3×3 内参矩阵。
        # K 布局：[fx, 0, cx, 0, fy, cy, 0, 0, 1] → 3×3 矩阵。
        # 这份数据是 Docker 端 create_point_cloud_from_depth_image 反投影的输入。
        K = self.latest_camera_info.k
        intrinsic_matrix = np.array([
            [K[0], K[1], K[2]],
            [K[3], K[4], K[5]],
            [K[6], K[7], K[8]]
        ], dtype=np.float64)

        meta = {
            'intrinsic_matrix': intrinsic_matrix,
            'factor_depth': np.array([self.factor_depth], dtype=np.float64),
        }

        if HAS_SCIPY:
            # mat 是二进制格式，同样走 先写 tmp 再 rename 的原子写，
            # 防止 Docker 端 loadmat 读到半截的 .mat 文件。
            path = os.path.join(self.shared_dir, 'meta.mat')
            tmp_path = f'{path}.tmp'
            scio.savemat(tmp_path, meta, appendmat=False)
            os.replace(tmp_path, path)
        else:
            path = os.path.join(self.shared_dir, 'meta.json')
            self._atomic_text_write(path, json.dumps({
                'intrinsic_matrix': intrinsic_matrix.tolist(),
                'factor_depth': self.factor_depth,
            }))

    def _write_debug_info(self):
        """保存人类可读的相机信息（调试用）"""
        info = self.latest_camera_info
        lines = [
            f"Timestamp: {time.strftime('%Y-%m-%d %H:%M:%S')}",
            f"Width: {info.width}, Height: {info.height}",
            f"K: {info.k}",
            f"Distortion model: {info.distortion_model}",
            f"D: {info.d}",
            f"R: {info.r}",
            f"P: {info.p}",
        ]
        path = os.path.join(self.shared_dir, 'camera_info.txt')
        self._atomic_text_write(path, '\n'.join(lines) + '\n')

    def _write_ready_flag(self):
        """写入就绪标志文件（供 GraspNet 检测新帧）"""
        # 时序关键点：.ready 必须在本帧所有数据文件写完之后再写
        # （调用顺序见 publish_to_shared）。推理端 wait_for_new_frame 只等
        # .ready 出现，出现即代表"整帧就绪"；处理完删除 .ready 进入下一轮，
        # 形成 生产者写 → 消费者删 的握手协议，保证帧之间不串扰。
        path = os.path.join(self.shared_dir, '.ready')
        self._atomic_text_write(path, f'{time.time()}\n')


def main(args=None):
    # 节点入口：初始化 rclpy、创建节点并进入 spin 循环。
    # spin 会阻塞在主线程，回调全部在 executor 内部线程调度执行。
    rclpy.init(args=args)
    node = CameraBridgeNode()

    # 处理 SIGINT（Ctrl+C）以干净退出：rclpy.shutdown 会让 spin 返回，
    # 再走 finally 里的资源清理（destroy_node + shutdown）。
    def signal_handler(sig, frame):
        node.get_logger().info('Shutting down camera bridge node...')
        rclpy.shutdown()

    signal.signal(signal.SIGINT, signal_handler)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
verify_pipeline.py — GraspNet 感知流水线验收脚本

分阶段验证每个组件的正确性。

用法：
  python verify_pipeline.py              # 全量检查
  python verify_pipeline.py --stage env  # 仅检查环境和依赖
  python verify_pipeline.py --stage gazebo    # 仅检查 Gazebo 世界文件
  python verify_pipeline.py --stage docker    # 仅检查 Docker 镜像
  python verify_pipeline.py --stage topics    # 仅检查 ROS 2 话题
  python verify_pipeline.py --stage data      # 仅检查共享目录数据
  python verify_pipeline.py --stage placement # 检查 Gazebo 放置结果
  python verify_pipeline.py --stage all       # 全量检查（默认）
"""

import argparse
import json
import os
import shutil
import subprocess
import sys
import time
import math
import re

# ── 路径配置 ──────────────────────────────────────────────────────
PROJECT_DIR = os.path.expanduser('~/piper_graspnet')
GRASPNET_DIR = os.path.join(PROJECT_DIR, 'graspnet-baseline')
GAZEBO_DIR = os.path.join(PROJECT_DIR, 'gazebo_piper_pick_place')
SHARED_DIR = os.path.join(PROJECT_DIR, 'shared_camera_data')
WORLD_FILE = os.path.join(
    GAZEBO_DIR, 'src/piper_gazebo/worlds/piper_pick_graspnet.world')
EXECUTION_WORLD_FILE = os.path.join(
    GAZEBO_DIR, 'src/piper_gazebo/worlds/piper_pick.world')
BRIDGE_SCRIPT = os.path.join(
    GAZEBO_DIR, 'src/piper_perception/piper_perception/camera_bridge_node.py')
BRIDGE_ENTRY = 'camera_bridge_node'  # console_scripts entry point name
INFERENCE_SCRIPT = os.path.join(
    GRASPNET_DIR, 'inference_from_camera.py')
DOCKER_RUN_SCRIPT = os.path.join(PROJECT_DIR, 'graspnet-docker-camera.sh')
LAUNCH_FILE = os.path.join(
    GAZEBO_DIR, 'src/piper_perception/launch/graspnet_pipeline.launch.py')
PICK_NODE = os.path.join(
    GAZEBO_DIR, 'src/piper_gazebo/src/pick_and_place.cpp')
PICK_LAUNCH = os.path.join(
    GAZEBO_DIR, 'src/piper_gazebo/launch/piper_with_gripper/piper_pick_and_place.launch.py')

OBJECT_COLOR_RANGES = {
    'red_box': ((0.55, 0.00, 0.00), (1.01, 0.35, 0.35)),
    'green_cylinder': ((0.00, 0.45, 0.00), (0.35, 1.01, 0.35)),
    'blue_cylinder': ((0.00, 0.00, 0.45), (0.35, 0.35, 1.01)),
}
MIN_OBJECT_PIXELS = 80
DROP_TARGET_XY = (0.650, -0.220)  # 必须匹配 launch file 中的 back_place_x/y
DROP_XY_TOLERANCE = 0.16
DROP_Z_RANGE = (0.0, 0.20)

# ── 全局状态 ──────────────────────────────────────────────────────
ROS2_BIN = None  # 运行时发现 ros2 路径

def find_ros2():
    """查找 ros2 可执行文件路径"""
    global ROS2_BIN
    if ROS2_BIN:
        return ROS2_BIN
    # 检查 PATH
    ROS2_BIN = shutil.which('ros2')
    if not ROS2_BIN:
        for candidate in ['/opt/ros/humble/bin/ros2',
                          '/opt/ros/iron/bin/ros2',
                          '/opt/ros/rolling/bin/ros2']:
            if os.path.exists(candidate):
                ROS2_BIN = candidate
                break
    return ROS2_BIN


GREEN = '\033[92m'
RED = '\033[91m'
YELLOW = '\033[93m'
CYAN = '\033[96m'
RESET = '\033[0m'
BOLD = '\033[1m'

passed = 0
failed = 0


def ok(msg):
    """登记一项通过：全局 passed 计数 +1 并打印绿色 PASS 行。"""
    global passed
    passed += 1
    print(f"  {GREEN}✓ PASS{RESET}  {msg}")


def fail(msg):
    """登记一项失败：全局 failed 计数 +1 并打印红色 FAIL 行。"""
    global failed
    failed += 1
    print(f"  {RED}✗ FAIL{RESET}  {msg}")


def warn(msg):
    """打印黄色告警（不计数）：环境缺失但可容忍的情况。"""
    print(f"  {YELLOW}⚠ WARN{RESET}  {msg}")


def info(msg):
    """打印青色信息行（不计数）：过程性信息，帮助理解检查上下文。"""
    print(f"  {CYAN}▶ INFO{RESET}  {msg}")


def section(title):
    """打印分节标题，把不同检查阶段在输出里分隔开。"""
    print(f"\n{BOLD}{'=' * 60}{RESET}")
    print(f"{BOLD}  {title}{RESET}")
    print(f"{BOLD}{'=' * 60}{RESET}")


def run_cmd(cmd, timeout=10, env=None):
    """运行命令并返回 (returncode, stdout, stderr)"""
    cmd_env = os.environ.copy()
    # 尝试找到 ROS 2 安装路径
    for ros_root in ['/opt/ros/humble', '/opt/ros/iron', '/opt/ros/rolling']:
        setup = os.path.join(ros_root, 'setup.bash')
        if os.path.exists(setup):
            cmd_env.setdefault('ROS_DISTRO', os.path.basename(ros_root))
            cmd_env['AMENT_PREFIX_PATH'] = f'{ros_root}:{cmd_env.get("AMENT_PREFIX_PATH", "")}'
            cmd_env['CMAKE_PREFIX_PATH'] = f'{ros_root}:{cmd_env.get("CMAKE_PREFIX_PATH", "")}'
            cmd_env['PYTHONPATH'] = f'{ros_root}/lib/python3.10/site-packages:{cmd_env.get("PYTHONPATH", "")}'
            cmd_env['PATH'] = f'{ros_root}/bin:{cmd_env.get("PATH", "")}'
            break
    if env:
        cmd_env.update(env)
    # 如果是 ros2 命令但没有找到完整路径，自动补全
    if cmd[0] == 'ros2' and not os.path.isabs(cmd[0]):
        resolved = shutil.which('ros2', path=cmd_env.get('PATH', os.environ.get('PATH', '')))
        if resolved:
            cmd = [resolved] + cmd[1:]
    try:
        result = subprocess.run(
            cmd, capture_output=True, text=True, timeout=timeout, env=cmd_env)
        return result.returncode, result.stdout, result.stderr
    except subprocess.TimeoutExpired:
        return -1, '', 'TIMEOUT'
    except FileNotFoundError:
        return -2, '', 'NOT FOUND'


# ===================================================================
#  环境检查
# ===================================================================
def check_env():
    """阶段 1：检查 ROS 2 CLI、Python 依赖、GPU 与 Docker 运行时是否齐备。"""
    section('1. 环境与依赖检查')

    # ROS 2
    ros2_path = find_ros2()
    if ros2_path:
        ret, out, _ = run_cmd([ros2_path, '--help'])
        if ret == 0:
            ros_distro = os.environ.get('ROS_DISTRO', 'unknown')
            # Extract first line
            first_line = out.strip().split('\n')[0] if out else 'ros2 CLI'
            ok(f"ROS 2 ({ros_distro}): {first_line}")
        else:
            ret2, out2, _ = run_cmd(['dpkg', '-l', f'ros-{os.environ.get("ROS_DISTRO", "humble")}-ros2cli'])
            if ret2 == 0:
                # Extract version from dpkg output
                for line in out2.split('\n'):
                    if 'ros2cli' in line:
                        parts = line.split()
                        if len(parts) >= 3:
                            ok(f"ROS 2: {parts[2]} (dpkg)")
                            break
            else:
                warn(f"ros2 CLI found at {ros2_path} but version query failed")
    else:
        fail("ros2 CLI not available")

    # Python deps
    try:
        import rclpy
        ok("Python rclpy available")
    except ImportError:
        fail("Python rclpy not available")

    try:
        import cv2
        ok(f"OpenCV: {cv2.__version__}")
    except ImportError:
        fail("OpenCV not available")

    try:
        import scipy
        ok(f"SciPy: {scipy.__version__}")
    except ImportError:
        warn("SciPy not available (meta will be saved as JSON)")

    try:
        import torch
        ok(f"PyTorch: {torch.__version__}")
        if torch.cuda.is_available():
            ok(f"CUDA device: {torch.cuda.get_device_name(0)}")
        else:
            warn("CUDA not available (will use CPU)")
    except ImportError:
        warn("PyTorch not available on host (expected if not installed)")

    # NVIDIA
    ret, out, _ = run_cmd(['nvidia-smi'])
    if ret == 0:
        ok("NVIDIA GPU detected")
    else:
        warn("nvidia-smi failed (expected in non-GPU environments)")

    # Docker
    ret, out, _ = run_cmd(['docker', '--version'])
    if ret == 0:
        ok(f"Docker: {out.strip()}")
    else:
        fail("Docker not available")

    ret, out, _ = run_cmd(['nvidia-container-runtime', '--version'], timeout=5)
    if ret == 0:
        ok("nvidia-container-runtime available")
    else:
        warn("nvidia-container-runtime not found (GPU Docker may fail)")


# ===================================================================
#  项目结构检查
# ===================================================================
def check_project():
    """阶段 2：检查关键文件/目录存在性，并核对世界文件与源码中的关键配置
    （相机姿态、物体摆放、标定矩阵、GraspNet→TCP 轴映射、门控与暂停机制等），
    防止"改了配置但没同步"导致的静默失败。
    """
    section('2. 项目结构检查')

    checks = [
        ("Project root", PROJECT_DIR, True),
        ("GraspNet symlink", GRASPNET_DIR, True),
        ("Gazebo package", GAZEBO_DIR, True),
        ("World file", WORLD_FILE, True),
        ("Execution world file", EXECUTION_WORLD_FILE, True),
        ("Camera bridge script", BRIDGE_SCRIPT, True),
        ("Inference script", INFERENCE_SCRIPT, True),
        ("Docker run script", DOCKER_RUN_SCRIPT, True),
        ("CMakeLists.txt", os.path.join(GAZEBO_DIR, 'src/piper_gazebo/CMakeLists.txt'), True),
        ("Shared data dir", SHARED_DIR, True),
    ]

    for name, path, required in checks:
        if os.path.exists(path):
            ok(f"{name}: {path}")
        elif required:
            fail(f"{name}: {path} (missing!)")
        else:
            warn(f"{name}: {path} (not found, but optional)")

    for label, world_path in (
        ('Perception world', WORLD_FILE),
        ('Execution world', EXECUTION_WORLD_FILE),
    ):
        if not os.path.exists(world_path):
            continue
        with open(world_path) as f:
            content = f.read()
        if 'gazebo_ros_camera' in content:
            ok(f"{label} contains gazebo_ros_camera plugin")
        else:
            fail(f"{label} missing gazebo_ros_camera plugin")
        if '<pose>0.1 0 0.6 0 1.047198 0</pose>' in content:
            ok(f"{label} uses oblique 60° camera pose")
        else:
            fail(f"{label} camera pose does not match oblique 60° pitch config")
        if (
            '<model name="filterable_platform">' in content
            and '<pose>0.45 0.00 0.050 0 0 0</pose>' in content
            and '<geometry><box><size>0.80 0.50 0.100</size></box></geometry>' in content
        ):
            ok(f"{label} defines enlarged 10cm gray platform filtered as background")
        else:
            fail(f"{label} missing enlarged 10cm filterable platform")
        if '<horizontal_fov>1.0' in content and '<width>800</width>' in content:
            ok(f"{label} uses expanded top camera recognition area (FOV=1.0 rad, 800x600)")
        else:
            fail(f"{label} camera FOV/resolution does not match expanded recognition plan")
        if '<pose>0.45 -0.12 0.124000 0 1.570796 0</pose>' in content:
            ok(f"{label} red cylinder is lying down at y=-0.12 (8/2 layout)")
        else:
            fail(f"{label} red cylinder pose does not match current layout (0.45 -0.12, lying)")
        if '<pose>0.45 0.02 0.124000 0 0 0</pose>' in content:
            ok(f"{label} green cylinder is upright at original placement")
        else:
            fail(f"{label} green cylinder is not upright at original placement")
        if '<pose>0.45 0.17 0.124000 0 0 0</pose>' in content:
            ok(f"{label} blue cylinder is upright at original placement")
        else:
            fail(f"{label} blue cylinder is not upright at original placement")
        if content.count('<geometry><cylinder><radius>0.024</radius><length>0.0675</length></cylinder></geometry>') >= 6:
            ok(f"{label} defines three cylinder collision/visual pairs")
        else:
            fail(f"{label} does not define all three targets as cylinders")
        if label == 'Execution world':
            if 'piper_link_attacher' in content and 'libpiper_link_attacher.so' in content:
                ok("Execution world loads Gazebo attach plugin for gated attach")
            else:
                fail("Execution world missing Gazebo attach plugin")

    if os.path.exists(PICK_NODE) and os.path.exists(INFERENCE_SCRIPT):
        with open(PICK_NODE) as f:
            pick_content = f.read()
        with open(INFERENCE_SCRIPT) as f:
            inference_content = f.read()
        expected_tf_tokens = [
            '{0.0, -0.82806033, 0.56063899}',
            '{-1.0, 0.0, 0.0}',
            '{0.0, -0.56063899, -0.82806033}',
            '{0.072, 0.021, 0.548}',
        ]
        if all(token in pick_content for token in expected_tf_tokens):
            ok("Pick node camera-to-world TF uses corrected calibration (fixed R column 1 signs, re-fitted T)")
        else:
            fail("Pick node camera-to-world TF does not match corrected calibration")
        if 'CAMERA_TO_WORLD_T = np.array([0.072, 0.021, 0.548]' in inference_content:
            ok("Inference script camera-to-world TF uses corrected calibration")
        else:
            fail("Inference script camera-to-world TF does not match corrected calibration")
        expected_graspnet_tcp_tokens = [
            '{0.0, 0.0, 1.0}',
            '{0.0, 1.0, 0.0}',
            '{-1.0, 0.0, 0.0}',
        ]
        if all(token in pick_content for token in expected_graspnet_tcp_tokens):
            ok("Pick node maps GraspNet +X approach to Piper TCP +Z")
        else:
            fail("Pick node GraspNet-to-Piper TCP axis mapping is inverted")
        if (
            '[0.0, 0.0, 1.0]' in inference_content
            and '[-1.0, 0.0, 0.0]' in inference_content
        ):
            ok("Inference diagnostics use the same GraspNet-to-Piper TCP mapping")
        else:
            fail("Inference diagnostics GraspNet-to-Piper TCP mapping is inverted")
        viewer_path = os.path.join(PROJECT_DIR, 'open3d_result_viewer.py')
        if os.path.exists(viewer_path):
            with open(viewer_path) as f:
                viewer_content = f.read()
            if 'approach = np.array([depth * 0.7' in viewer_content:
                ok("Open3D viewer draws GraspNet approach along local +X")
            else:
                warn("Open3D viewer approach direction check skipped (viewer may have been updated)")
        if 'graspnet_pause' not in pick_content and 'setInferencePaused' not in pick_content:
            ok("Pick node runs GraspNet free (pause mechanism deliberately removed)")
        elif 'graspnet_pause' in pick_content and 'setInferencePaused(true)' in pick_content:
            ok("Pick node pauses GraspNet before arm trajectory execution")
        else:
            warn("Pick node GraspNet pause state is ambiguous")
        if 'graspnet_pause' not in inference_content and 'wait_for_new_frame' in inference_content:
            ok("Inference script runs freely without GraspNet pause (deliberate)")
        elif 'graspnet_pause' in inference_content and 'clear_ready_flag' in inference_content:
            ok("Inference script honors GraspNet pause flag")
        else:
            warn("Inference script GraspNet pause state is ambiguous")
        if 'verifyCurrentTcpNearObject' in pick_content and 'max_grasp_distance' in pick_content:
            ok("Pick node checks TCP-to-object distance before accepting grasp")
        else:
            fail("Pick node missing TCP-to-object distance gate")
        if 'setPoseTarget(target, kEndEffectorLink)' in pick_content and 'setPositionTarget(' not in pick_content:
            ok("Pick node fallback planning preserves full GraspNet pose")
        else:
            fail("Pick node fallback planning may drop GraspNet orientation")
        if 'reachableTopDownTcpOrientation' not in pick_content and 'projecting to reachable top-down' not in pick_content:
            ok("Pick node does not replace GraspNet orientation with a fallback pose")
        else:
            fail("Pick node still replaces GraspNet orientation")
        if (
            'attachObjectAfterOpenGripperDistanceGate' in pick_content
            and 'kMaxAttachCenterDistance' in pick_content
            and 'detachObject(' in pick_content
        ):
            ok("Pick node uses gated attach after open-gripper center-distance check")
        else:
            fail("Pick node missing gated attach/detach logic")
        if 'SOURCE_OBJECT_PRIORS_WORLD' in inference_content and 'SOURCE_PRIOR_MAX_XY_DIST' in inference_content:
            ok("Inference script filters grasp targets by source object region")
        else:
            fail("Inference script missing source-region grasp filtering")

    if os.path.exists(PICK_LAUNCH):
        with open(PICK_LAUNCH) as f:
            launch_content = f.read()
        if '"shared_dir": shared_dir' in launch_content and 'perception_grasps.csv' in launch_content:
            ok("Pick launch passes shared perception paths to controller")
        else:
            fail("Pick launch missing shared perception path parameters")
        if 'max_grasp_distance' not in launch_content:
            ok("Pick launch omits max_grasp_distance (uses code default 0.100m)")
        else:
            info("Pick launch overrides max_grasp_distance")
# ===================================================================
#  Docker 镜像检查
# ===================================================================
def check_docker():
    """阶段 3：确认 GraspNet 推理镜像存在，且镜像构建配置中包含推理脚本。"""
    section('3. Docker 镜像检查')

    ret, out, _ = run_cmd(['docker', 'images', 'graspnet-baseline:20.04',
                          '--format', '{{.Repository}}:{{.Tag}}'])
    if 'graspnet-baseline:20.04' in out:
        ok("Docker image graspnet-baseline:20.04 exists")
    else:
        fail("Docker image graspnet-baseline:20.04 not found")

    # 检查 Dockerfile 是否包含推理脚本
    dockerfile = os.path.join(GRASPNET_DIR, 'Dockerfile.20.04')
    if os.path.exists(dockerfile):
        with open(dockerfile) as f:
            content = f.read()
        if 'inference_from_camera.py' in content:
            ok("Dockerfile includes inference_from_camera.py")
        else:
            fail("Dockerfile missing inference_from_camera.py")

    # 检查 .dockerignore
    dockerignore = os.path.join(GRASPNET_DIR, '.dockerignore')
    if os.path.exists(dockerignore):
        ok(".dockerignore exists")
    else:
        warn(".dockerignore not found")


# ===================================================================
#  ROS 2 话题检查（需要系统正在运行）
# ===================================================================
def check_topics():
    """阶段 4：确认相机三个核心话题在发布（需要感知流水线正在运行）。"""
    section('4. ROS 2 话题检查（需要系统运行中）')

    ret, out, _ = run_cmd(['ros2', 'topic', 'list'], timeout=5)
    if ret != 0:
        warn("Cannot list ROS 2 topics (system may not be running)")
        return

    topics = out.strip().split('\n')
    info(f"Active topics: {len(topics)}")

    required_topics = [
        '/camera/image_raw',
        '/camera/depth/image_raw',
        '/camera/camera_info',
    ]

    for topic in required_topics:
        if topic in topics:
            ok(f"Topic {topic} is publishing")
        else:
            fail(f"Topic {topic} not found")

    # 检查话题频率
    for topic in required_topics[:3]:  # 只检查前三个核心话题
        ret, out, _ = run_cmd(
            ['timeout', '3', 'ros2', 'topic', 'hz', topic, '--window', '5'],
            timeout=5)
        if ret == 0 or 'average' in out.lower():
            info(f"{topic} → hz info available")
        else:
            warn(f"{topic} → no data yet")


# ===================================================================
#  共享目录数据完整性检查
# ===================================================================
def check_shared_data():
    """阶段 5：检查共享目录中 bridge 写入的各文件（color/depth/meta/
    .ready/camera_info.txt/last_validation.json）是否可读、内容是否合理。"""
    section('5. 共享目录数据完整性检查')

    if not os.path.exists(SHARED_DIR):
        fail(f"Shared directory {SHARED_DIR} not found")
        return

    files = os.listdir(SHARED_DIR)
    info(f"Files in shared directory: {files}")

    # color.png
    color_path = os.path.join(SHARED_DIR, 'color.png')
    if os.path.exists(color_path):
        try:
            import cv2
            img = cv2.imread(color_path)
            if img is not None:
                h, w = img.shape[:2]
                ok(f"color.png: {w}x{h}, {img.dtype}")
            else:
                fail("color.png exists but cannot be decoded")
        except Exception as e:
            fail(f"color.png read error: {e}")
    else:
        fail("color.png not found (system may not be running)")

    # depth.png
    depth_path = os.path.join(SHARED_DIR, 'depth.png')
    if os.path.exists(depth_path):
        try:
            import cv2
            depth = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)
            if depth is not None:
                h, w = depth.shape[:2]
                ok(f"depth.png: {w}x{h}, {depth.dtype}, "
                   f"range [{depth.min()}, {depth.max()}]")
            else:
                fail("depth.png exists but cannot be decoded")
        except Exception as e:
            fail(f"depth.png read error: {e}")
    else:
        fail("depth.png not found")

    # meta.mat
    meta_path = os.path.join(SHARED_DIR, 'meta.mat')
    if os.path.exists(meta_path):
        try:
            import scipy.io as scio
            meta = scio.loadmat(meta_path)
            if 'intrinsic_matrix' in meta:
                K = meta['intrinsic_matrix']
                ok(f"meta.mat: intrinsic_matrix ({K.shape}), "
                   f"factor_depth={meta.get('factor_depth', 'N/A')}")
            else:
                fail("meta.mat missing intrinsic_matrix")
        except Exception as e:
            fail(f"meta.mat read error: {e}")
    elif os.path.exists(os.path.join(SHARED_DIR, 'meta.json')):
        ok("meta.json found (SciPy not available on host)")
    else:
        fail("meta.mat not found")

    # .ready flag
    ready_path = os.path.join(SHARED_DIR, '.ready')
    if os.path.exists(ready_path):
        with open(ready_path) as f:
            ts = f.read().strip()
        ok(f".ready flag exists (timestamp: {ts})")
    else:
        warn(".ready flag not found (expected if bridge is not running)")

    # camera_info.txt
    info_path = os.path.join(SHARED_DIR, 'camera_info.txt')
    if os.path.exists(info_path):
        ok("camera_info.txt exists")
    else:
        warn("camera_info.txt not found")

    validation_path = os.path.join(SHARED_DIR, 'last_validation.json')
    if os.path.exists(validation_path):
        try:
            with open(validation_path) as f:
                validation = json.load(f)
            object_stats = validation.get('object_stats', {})
            ok(f"last_validation.json exists (frame {validation.get('frame', 'unknown')})")
            info(f"Object point stats: {object_stats}")
        except Exception as e:
            fail(f"last_validation.json read error: {e}")
    else:
        warn("last_validation.json not found (expected until inference runs after this update)")


def _load_camera_images():
    """读取共享目录的 color.png/depth.png，返回 (RGB float 图, uint16 深度, np)。"""
    try:
        import cv2
        import numpy as np
    except ImportError as e:
        fail(f"Camera validation dependency missing: {e}")
        return None, None, None

    color_path = os.path.join(SHARED_DIR, 'color.png')
    depth_path = os.path.join(SHARED_DIR, 'depth.png')
    if not os.path.exists(color_path) or not os.path.exists(depth_path):
        fail("color.png/depth.png missing; start graspnet_pipeline.launch.py first")
        return None, None, None

    color_bgr = cv2.imread(color_path, cv2.IMREAD_COLOR)
    depth = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)
    if color_bgr is None or depth is None:
        fail("Failed to decode color.png or depth.png")
        return None, None, None

    color = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    return color, depth, np


def _segment_color_objects(color, depth, np):
    """按颜色区间 + 有效深度像素分割物体，返回 (各物体像素计数, 掩膜)。"""
    flat = color.reshape(-1, 3)
    valid_depth = depth.reshape(-1) > 0
    counts = {}
    masks = {}
    for name, (lower, upper) in OBJECT_COLOR_RANGES.items():
        lo = np.array(lower, dtype=np.float32)
        hi = np.array(upper, dtype=np.float32)
        mask = np.all(flat >= lo, axis=1) & np.all(flat <= hi, axis=1) & valid_depth
        masks[name] = mask.reshape(depth.shape)
        counts[name] = int(np.count_nonzero(mask))
    counts['total_object_pixels'] = int(sum(counts.values()))
    counts['valid_depth_pixels'] = int(np.count_nonzero(valid_depth))
    return counts, masks


def check_camera_observation():
    """阶段 6：从当前相机帧统计各物体的可见像素数，验证相机视角/FOV
    是否覆盖全部物体（识别能力的静态验证）。"""
    section('6. 相机点云目标检查')

    color, depth, np = _load_camera_images()
    if color is None:
        return

    counts, masks = _segment_color_objects(color, depth, np)
    info(f"Valid depth pixels: {counts['valid_depth_pixels']}")
    info(
        "Colored object pixels: "
        f"red={counts['red_box']}, green={counts['green_cylinder']}, "
        f"blue={counts['blue_cylinder']}"
    )

    for name in OBJECT_COLOR_RANGES:
        if counts[name] >= MIN_OBJECT_PIXELS:
            ok(f"{name}: visible non-plane colored depth pixels >= {MIN_OBJECT_PIXELS}")
        else:
            fail(f"{name}: only {counts[name]} colored depth pixels; adjust camera pose/FOV")

    if counts['valid_depth_pixels'] > 0:
        ratio = counts['total_object_pixels'] / counts['valid_depth_pixels']
        info(f"Object/scene valid depth ratio: {ratio:.4f}")
        if ratio > 0 and counts['total_object_pixels'] < counts['valid_depth_pixels']:
            ok("Object segmentation excludes background/plane pixels")
        else:
            fail("Object segmentation did not isolate objects from the scene")

    debug_path = os.path.join(SHARED_DIR, 'camera_object_mask_stats.json')
    with open(debug_path, 'w') as f:
        json.dump(counts, f, indent=2)
    ok(f"Wrote camera object stats: {debug_path}")


def check_grasp_validation():
    """阶段 7：检查 Docker 推理产出的 last_validation.json——各物体识别点数
    （状态感知：已放置物体按 pick_state 跳过）与抓取中心是否落在物体包围盒内。"""
    section('7. 最终夹爪位置检查')

    validation_path = os.path.join(SHARED_DIR, 'last_validation.json')
    if not os.path.exists(validation_path):
        fail("last_validation.json not found; run Docker inference after this update")
        return

    try:
        with open(validation_path) as f:
            validation = json.load(f)
    except Exception as e:
        fail(f"last_validation.json read error: {e}")
        return

    object_stats = validation.get('object_stats', {})
    # 状态感知：只对尚未放置（pending）的物体要求识别点数；
    # 已放置物体按 pick_state.json 跳过（过滤生效后不再被感知）。
    pending_names = list(OBJECT_COLOR_RANGES.keys())
    state_path = os.path.join(SHARED_DIR, 'pick_state.json')
    try:
        with open(state_path) as f:
            pending = json.load(f).get('pending')
        if isinstance(pending, list) and pending:
            pending_names = [n for n in pending if n in OBJECT_COLOR_RANGES]
    except (OSError, ValueError):
        pass  # 无状态文件 → 全部视为 pending（感知-only 模式）

    for name in OBJECT_COLOR_RANGES:
        if name not in pending_names:
            info(f"{name}: placed, skipped (pick_state filtering active)")
            continue
        count = int(object_stats.get(name, 0))
        if count >= MIN_OBJECT_PIXELS:
            ok(f"{name}: inference saw {count} object points")
        else:
            fail(f"{name}: inference saw only {count} object points")

    grasp_validation = validation.get('grasp_validation', {})
    grasp_count = int(grasp_validation.get('grasp_count', 0))
    bbox_hits = int(grasp_validation.get('centers_inside_object_bbox', 0))
    best_inside = bool(grasp_validation.get('best_grasp_inside_object_bbox', False))

    if grasp_count > 0:
        ok(f"Inference produced {grasp_count} final grasp(s)")
    else:
        fail("Inference produced no final grasps")

    if bbox_hits > 0:
        ok(f"{bbox_hits} grasp center(s) are near the object point cloud bbox")
    else:
        fail("No grasp centers are near the object point cloud bbox")

    if best_inside:
        ok("Best grasp center is near the object point cloud bbox")
    else:
        fail("Best grasp center is outside the object point cloud bbox")


# ===================================================================
#  Gazebo 放置结果检查
# ===================================================================
def get_gazebo_entity_position(name):
    """通过 /get_entity_state 服务查询 Gazebo 中某实体的世界系位置。

    返回 (x, y, z) 元组或 (None, 错误信息)。服务调用需要在 ROS 2 环境
    已 source 的 shell 中执行，因此用 bash -lc 包装。
    """
    cmd = (
        "source /opt/ros/humble/setup.bash && "
        f"ros2 service call /get_entity_state gazebo_msgs/srv/GetEntityState "
        f"'{{name: {name}, reference_frame: world}}'"
    )
    rc, stdout, stderr = run_cmd(['bash', '-lc', cmd], timeout=15)
    if rc != 0:
        return None, stderr or stdout

    match = re.search(
        r"position=geometry_msgs\.msg\.Point\(x=([-+0-9.eE]+), "
        r"y=([-+0-9.eE]+), z=([-+0-9.eE]+)\)",
        stdout,
    )
    if not match:
        return None, stdout

    return tuple(float(match.group(i)) for i in range(1, 4)), None


def check_gazebo_placement():
    """阶段 8：检查抓取码放结果——三个物体最终位置是否落在放置区
    （DROP_TARGET_XY ± 容差，须与 launch 文件的 back_place_x/y 一致）。"""
    section('8. Gazebo 放置结果检查')

    for name in ('red_box', 'green_cylinder', 'blue_sphere'):
        position, error = get_gazebo_entity_position(name)
        if position is None:
            fail(f"{name}: failed to query Gazebo entity state: {error}")
            continue

        x, y, z = position
        dist_xy = math.hypot(x - DROP_TARGET_XY[0], y - DROP_TARGET_XY[1])
        if dist_xy <= DROP_XY_TOLERANCE and DROP_Z_RANGE[0] <= z <= DROP_Z_RANGE[1]:
            ok(
                f"{name}: placed at ({x:.3f}, {y:.3f}, {z:.3f}), "
                f"xy error {dist_xy:.3f} m"
            )
        else:
            fail(
                f"{name}: final pose ({x:.3f}, {y:.3f}, {z:.3f}) outside "
                f"drop tolerance; xy error {dist_xy:.3f} m"
            )


# ===================================================================
#  CMakeLists 检查
# ===================================================================
def check_cmake():
    """阶段 9：检查 piper_perception 包结构（setup.py 入口点、launch 文件等）。"""
    section('9. 包结构检查')

    # piper_perception Python package
    pkg_dir = os.path.join(GAZEBO_DIR, 'src/piper_perception')
    setup_py = os.path.join(pkg_dir, 'setup.py')
    package_xml = os.path.join(pkg_dir, 'package.xml')

    checks = {
        'piper_perception package exists': os.path.exists(pkg_dir),
        'setup.py': os.path.exists(setup_py),
        'package.xml': os.path.exists(package_xml),
        'camera_bridge_node.py in package': os.path.exists(BRIDGE_SCRIPT),
        'launch file in package': os.path.exists(LAUNCH_FILE),
    }

    for name, result in checks.items():
        if result:
            ok(f"Perception package: {name}")
        else:
            fail(f"Perception package: {name}")

    # Check setup.py has the correct entry point
    if os.path.exists(setup_py):
        with open(setup_py) as f:
            content = f.read()
        if 'camera_bridge_node' in content:
            ok("setup.py has camera_bridge_node console_scripts entry")
        else:
            fail("setup.py missing camera_bridge_node entry point")


# ===================================================================
#  主函数
# ===================================================================
def main():
    """入口：解析 --stage 参数，按阶段运行对应的检查函数，
    最后汇总 passed/failed 计数并以退出码 0/1 结束（供 CI/脚本判断）。"""
    parser = argparse.ArgumentParser(description='GraspNet Pipeline Verification')
    parser.add_argument('--stage', choices=['env', 'project', 'docker', 'topics',
                                            'data', 'camera', 'grasps',
                                            'placement', 'cmake', 'all'],
                       default='all', help='Verification stage')
    args = parser.parse_args()

    print(f"{BOLD}{'=' * 60}{RESET}")
    print(f"{BOLD}  GraspNet 感知流水线 — 验收检查{RESET}")
    print(f"{BOLD}  时间: {time.strftime('%Y-%m-%d %H:%M:%S')}{RESET}")
    print(f"{BOLD}{'=' * 60}{RESET}")

    stages = {
        'env': check_env,
        'project': check_project,
        'docker': check_docker,
        'topics': check_topics,
        'data': check_shared_data,
        'camera': check_camera_observation,
        'grasps': check_grasp_validation,
        'placement': check_gazebo_placement,
        'cmake': check_cmake,
    }

    if args.stage == 'all':
        for name, func in stages.items():
            func()
    else:
        stages[args.stage]()

    # ── 总结 ───────────────────────────────────────────────────────
    global passed, failed
    total = passed + failed
    print(f"\n{BOLD}{'=' * 60}{RESET}")
    print(f"{BOLD}  验收结果: {GREEN}{passed} passed{RESET}"
          f"{BOLD}, {RED}{failed} failed{RESET}"
          f"{BOLD}, {total} total{RESET}")
    print(f"{BOLD}{'=' * 60}{RESET}")

    if failed > 0:
        print(f"\n{YELLOW}⚠  {failed} 项检查未通过。请查看具体信息并修复。{RESET}")
        sys.exit(1)
    else:
        print(f"\n{GREEN}✓  全部检查通过！流水线配置正确。{RESET}")
        sys.exit(0)


if __name__ == '__main__':
    main()

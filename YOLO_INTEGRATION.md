# YOLOv8s-seg → RGB-D 点云 → GraspNet 集成说明

## 完成内容

- YOLOv8s-seg 微调、数据校验和 ONNX 导出脚本；
- ROS 2 `piper_yolo/yolo_bridge_node`；
- RGB、深度的近似时间同步和相机内参订阅；
- 不依赖 PyTorch/Ultralytics 的 ONNX Runtime 推理；
- 二维实例掩膜到 GraspNet 目标三维点云的桥接；
- 实例标签贯穿 GraspNet 候选 CSV 和抓取状态机；
- `.ready` 握手、原子文件替换和状态版本控制；
- 旧颜色阈值分割作为调试回退模式。

运行模型固定为 `models/piper_yolov8s_seg.onnx`。当前 test 集包含 300 张图像、557 个实例；框 mAP50-95 为 0.961，掩膜 mAP50-95 为 0.724。各类掩膜 mAP50-95：cube 0.755、cuboid 0.728、cylinder 0.690。

## 数据流

```text
/camera/image_raw ───────┐
/camera/depth/image_raw ─┼─ 近似时间同步 ─ YOLOv8s-seg ONNX ─ instances.npz
/camera/camera_info ─────┘                         │
                                                  ▼
color.png + depth.png + meta.mat + 二维实例掩膜
                                                  │ 同一像素 (u,v)
                                                  ▼
X=(u-cx)Z/fx, Y=(v-cy)Z/fy, Z=depth/factor_depth
                                                  │
                                                  ▼
目标三维点云 ─ GraspNet ─ 碰撞过滤 ─ perception_grasps.csv
                                                  │
                                                  ▼
快速 IK → approach/grasp/lift/place 预检 → 抓取执行
```

YOLO 不生成三维点云。它输出与 RGB 图同尺寸的二维布尔掩膜；深度图给出每个像素的 Z，相机内参给出 `fx/fy/cx/cy`。RGB、深度和掩膜像素对齐后，GraspNet 推理入口先反投影深度图，再用实例掩膜索引出目标点。标签来自 YOLO，三维几何来自 RGB-D。

## 类别约定

类别编号必须始终保持：

```text
0 cube
1 cuboid
2 cylinder
```

类别描述几何形状，不依赖红、绿、蓝颜色。旧 Gazebo Classic 场景仍使用历史模型名，不能与轻量链路的 `cube/cuboid/cylinder` 执行器混用。

## 训练与导出

每个物体必须使用 YOLO segmentation 多边形标签，不能只标矩形框。复制配置模板并填写数据集路径：

```bash
cp yolo_training/piper_objects.yaml.example yolo_training/piper_objects.yaml
python3 -m venv .venv-yolo-train
source .venv-yolo-train/bin/activate
pip install -r yolo_training/requirements-training.txt

python yolo_training/train_yolov8s_seg.py \
  --data yolo_training/piper_objects.yaml \
  --epochs 150 --batch 16 --device 0
```

脚本自动选择 `best.pt` 并导出固定 640×640、batch=1、opset 17 的 ONNX。运行节点只读取 ONNX；`.pt` 仅用于续训，默认不提交。

## ROS 2 构建

```bash
./yolo_training/setup_runtime_env.sh
source /opt/ros/humble/setup.bash
cd gazebo_piper_pick_place
colcon build --packages-up-to piper_lightweight_sim --symlink-install
source install/setup.bash
```

节点会自动从仓库定位模型与共享目录。也可覆盖：

```bash
export PIPER_GRASP_ROOT="$(git rev-parse --show-toplevel)"
export PIPER_YOLO_MODEL="${PIPER_GRASP_ROOT}/models/piper_yolov8s_seg.onnx"
export PIPER_SHARED_DIR="${PIPER_GRASP_ROOT}/shared_camera_data"
```

## 共享文件协议

YOLO 节点生成：

- `color.png`：BGR 彩色图；
- `depth.png`：`uint16` 毫米深度；
- `meta.mat`：内参矩阵和 `factor_depth`；
- `instances.npz`：`masks/class_ids/scores/boxes_xyxy/labels`；
- `detections.json`：可读的检测结果；
- `yolo_overlay.png`：掩膜、框、标签叠加预览；
- `frame_manifest.json`：帧号、状态版本、时间戳和图像尺寸；
- `.ready`：最后写入的提交标志。

节点先将每个文件写到同目录临时文件，再原子替换正式文件，最后才创建 `.ready`。GraspNet 处理完成后删除 `.ready`，防止颜色、深度和掩膜跨帧混用。

## 状态驱动感知

抓取执行器原子更新 `pick_state.json`：

- `phase=perceiving` 且 `perception_enabled=true` 时允许一次新推理；
- `active_target` 只保留当前目标类别；
- `state_version` 保证同一次请求最多提交一帧；
- 规划和机械臂运动期间暂停 YOLO/GraspNet，减少遮挡误识别和 GPU 浪费；
- 成功物体加入完成列表，后续不再生成该目标候选。

独立验证 YOLO 时，可在轻量启动文件中设置 `state_driven_perception:=false`。

## 参数调整

基础参数位于 `piper_yolo/config/yolo_bridge.yaml`。常见调整：

- `confidence_threshold`：模型置信度门限；
- `iou_threshold`：NMS 门限；
- `mask_erode_pixels`：掩膜边缘混入背景时从 1 调到 2，小物体被腐蚀时设为 0；
- `max_detections`：单帧最大实例数；
- `min_publish_interval`：最低发布间隔；
- `single_instance_per_label`：同类仅保留一个实例。

GraspNet 的颜色阈值回退可通过 `--segmentation_source color` 启用，只用于对比调试。

## 验收顺序

1. 查看 `yolo_overlay.png`，确认类别、框和掩膜；
2. 检查 `instances.npz` 的掩膜尺寸与 `depth.png` 完全一致；
3. 查看 GraspNet 日志中当前类别的三维点数，应大于 0；
4. 检查 `last_validation.json` 和 `perception_grasps.csv`；
5. 先使用 `dry_run:=true` 完成四段预检；
6. 再执行单物体真实仿真抓取，最后测试三物体循环。

# YOLOv8s-seg 数据集、微调与 ONNX 导出

本项目使用实例分割标签，而不是只有矩形框的检测标签。每个目标需要标注轮廓多边形，单行格式为：

```text
class_id x1 y1 x2 y2 ... xn yn
```

所有坐标按图像宽高归一化到 0～1。推荐的数据集布局：

```text
datasets/piper_objects/
├── images/{train,val,test}/
└── labels/{train,val,test}/
```

类别编号必须固定为 `0 cube`、`1 cuboid`、`2 cylinder`。训练脚本会检查类别顺序、三个数据划分和 segmentation 多边形格式，发现检测框格式或类别重排时直接停止。

## 配置数据集

```bash
cd piper_with_yolo_graspnet
cp yolo_training/piper_objects.yaml.example yolo_training/piper_objects.yaml
```

编辑 `piper_objects.yaml` 的 `path`。可以使用相对于该 YAML 文件的路径，也可以在训练命令中通过 `--data` 指定其他配置。`piper_objects.yaml` 和实际训练集均不会提交到 Git。

## 创建训练环境

```bash
python3 -m venv .venv-yolo-train
source .venv-yolo-train/bin/activate
python -m pip install --upgrade pip
python -m pip install -r yolo_training/requirements-training.txt
```

训练环境与 GraspNet Docker 分开，避免 Ultralytics 更改 GraspNet 所需的 PyTorch/CUDA 版本。

## 微调并导出

```bash
python yolo_training/train_yolov8s_seg.py \
  --data yolo_training/piper_objects.yaml \
  --base-model yolov8s-seg.pt \
  --epochs 150 --imgsz 640 --batch 16 --device 0
```

脚本会：

1. 校验数据集和类别；
2. 微调 YOLOv8s-seg；
3. 选择训练目录中的 `best.pt`；
4. 保存训练检查点到 `models/piper_yolov8s_seg_best.pt`；
5. 导出固定 batch=1、640×640、opset 17 的 `models/piper_yolov8s_seg.onnx`；
6. 写入不包含开发机绝对目录的训练元数据。

ROS 2 节点只加载 ONNX，不需要 Ultralytics 或 PyTorch。

## 当前模型指标

- 请求训练 100 轮，第 65 轮早停，第 45 轮为最佳检查点；
- 验证集：box mAP50-95 0.960，mask mAP50-95 0.751；
- 独立 test 集：box mAP50-95 0.961，mask mAP50-95 0.724；
- 详细指标见 `models/piper_yolov8s_seg.test.json`。

## 安装推理环境

```bash
./yolo_training/setup_runtime_env.sh
```

该脚本建立 `.venv-yolo-runtime`，安装与 ROS Humble 的 NumPy/cv_bridge 兼容的 ONNX Runtime 环境。ROS 节点入口发现此环境后会自动使用它。

## 续训

```bash
python yolo_training/train_yolov8s_seg.py \
  --data yolo_training/piper_objects.yaml \
  --base-model runs/segment/piper_yolov8s_seg/weights/last.pt \
  --resume
```

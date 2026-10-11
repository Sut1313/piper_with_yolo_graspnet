# 模型目录

运行时模型固定命名为 `piper_yolov8s_seg.onnx`。它由
`yolo_training/train_yolov8s_seg.py` 自动生成，YOLO ROS2 节点启动时直接加载。

当前目录同时保留：

- `piper_yolov8s_seg_best.pt`：最佳训练检查点，仅用于续训/重新导出；
- `piper_yolov8s_seg.training.json`：训练参数、验证指标和 SHA-256；
- `piper_yolov8s_seg.test.json`：独立 test 集指标。

运行所需的 `piper_yolov8s_seg.onnx` 随仓库提交，因此完整克隆后可直接加载。
训练检查点 `.pt` 体积较大且只用于继续训练，仍由 `.gitignore` 排除。

当前 ONNX 的 SHA-256 为
`bd98dd453af0053a8b1cbafaf09327abb9665925871a6b5c4e15acbbc05258d6`。

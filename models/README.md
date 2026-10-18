# 模型目录

真机 `real_yolo_pointcloud_node` 默认加载 `tabletop_yolov8s_seg_v2.onnx`。
这是 16 类桌面物体实例分割模型，类别顺序为：

```text
pen, pencil, marker, eraser, ruler, scissors, stapler, tape,
sharpener, block, cup, bottle, book, phone, mouse, remote
```

`piper_yolov8s_seg.onnx` 是原来的 `cube/cuboid/cylinder` 三类模型，继续由
仿真 `yolo_bridge_node` 和轻量 Gazebo 抓取流程使用。两套节点的默认模型和
标签相互隔离。

当前目录同时保留：

- `piper_yolov8s_seg_best.pt`：最佳训练检查点，仅用于续训/重新导出；
- `piper_yolov8s_seg.training.json`：训练参数、验证指标和 SHA-256；
- `piper_yolov8s_seg.test.json`：独立 test 集指标。

两个 ONNX 模型均随仓库提交，因此完整克隆后可直接加载。
训练检查点 `.pt` 体积较大且只用于继续训练，仍由 `.gitignore` 排除。

当前 ONNX 的 SHA-256 为
`bd98dd453af0053a8b1cbafaf09327abb9665925871a6b5c4e15acbbc05258d6`。

16 类 v2 ONNX 的 SHA-256 为
`db8b059f17566202870a89f88e835cad19e76b45cbda790cfd59500df5c16620`。

`tabletop_yolov8s_seg_v2.metadata.json` 记录了类别、输入尺寸、ONNX opset、
校验和和模型许可信息。当前该模型标记为 `AGPL-3.0`；分发、
修改或集成到其他产品前，请按对应许可条款进行合规检查。

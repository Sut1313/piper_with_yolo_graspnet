# GraspNet Docker 与 Demo 使用教程

本文档记录当前仓库的 Docker 镜像构建、容器启动、demo 推理保存结果，以及本地主机查看结果的方法。

## 1. 构建 Docker 镜像

进入项目根目录：

```bash
cd graspnet-baseline
```

执行构建：

```bash
./docker-build.sh
```

默认生成镜像：

```bash
graspnet-baseline:20.04
```

当前 Dockerfile 使用的基础镜像是：

```text
nvcr.io/nvidia/pytorch:22.03-py3
```

该基础镜像已经内置 PyTorch/CUDA，避免构建时再从 `download.pytorch.org` 下载很慢的 PyTorch CUDA wheel。

## 2. 启动 Docker 交互环境

如果只想进入容器调试：

```bash
docker run --rm -it \
  --gpus all \
  --ipc=host \
  -v "$(pwd)/outputs:/workspace/outputs:rw" \
  graspnet-baseline:20.04 \
  bash
```

进入容器后检查 GPU：

```bash
python -c "import torch; print(torch.cuda.is_available(), torch.__version__, torch.version.cuda)"
```

如果输出中 `torch.cuda.is_available()` 为 `True`，说明容器内 GPU 可用。

## 3. 运行 demo 并保存结果到主机

推荐方式是不实时显示 Open3D 窗口，而是直接保存 PCD/PLY/NPY 文件到主机。

最快运行方式：

```bash
NUM_POINT=10000 COLLISION_THRESH=-1 ./docker-run-demo.sh save
```

默认完整点数运行：

```bash
./docker-run-demo.sh save
```

启用碰撞检测：

```bash
COLLISION_THRESH=0.01 ./docker-run-demo.sh save
```

输出目录：

```text
graspnet-baseline/outputs
```

生成文件：

```text
outputs/scene_cloud.pcd          # 原始场景点云
outputs/scene_with_grasps.pcd    # 场景点云 + 抓取器顶点点云
outputs/top_grasps.ply           # Top-50 抓取器 mesh
outputs/grasps.npy               # 抓取预测数组
```

注意：不要随便修改 `NUM_VIEW`。当前 checkpoint 是按 `num_view=300` 训练的。如果改小，例如 `NUM_VIEW=60`，会出现 checkpoint 权重维度不匹配。

## 4. 本地主机查看保存结果

本地查看脚本：

```bash
view_saved_result.py
```

如果本机没有安装 Open3D：

```bash
sudo apt update
sudo apt install -y python3-pip
python3 -m pip install --user -i https://pypi.tuna.tsinghua.edu.cn/simple open3d "numpy<2"
```

推荐查看 mesh 模式：

```bash
python3 view_saved_result.py --output_dir outputs --mode mesh --point_size 1.5
```

其他查看方式：

```bash
# 场景 + 抓取器合并点云
python3 view_saved_result.py --output_dir outputs --mode combined_cloud

# 只看场景点云
python3 view_saved_result.py --output_dir outputs --mode scene_only
```

## 5. 实时显示模式

如果确实要在容器里直接打开 Open3D 窗口：

```bash
./docker-run-demo.sh gui
```

该模式要求：

- 主机有图形桌面；
- `$DISPLAY` 环境变量存在；
- X11 允许 Docker 访问；
- 容器能访问 `/tmp/.X11-unix`。

当前更推荐 `save` 模式，因为它稳定、快、不依赖 Xvfb/Open3D 离屏渲染。

## 6. 常见问题

### demo 为什么慢？

默认 demo 的慢点通常不是神经网络推理，而是后处理：

- `collision_detection()` 是 CPU/Numpy/Open3D 后处理，复杂度与候选抓取数和场景点数相关；
- `NUM_POINT=20000` 输入点较多；
- Open3D headless/Xvfb 渲染可能卡住或不退出；
- 首次运行会有 CUDA 初始化开销。

如果只需要快速得到结果文件，推荐：

```bash
NUM_POINT=10000 COLLISION_THRESH=-1 ./docker-run-demo.sh save
```

### checkpoint 报 size mismatch 怎么办？

不要修改 `NUM_VIEW`。当前权重需要 `num_view=300`：

```bash
./docker-run-demo.sh save
```

或者显式指定：

```bash
NUM_VIEW=300 ./docker-run-demo.sh save
```

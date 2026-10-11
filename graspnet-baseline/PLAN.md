# GraspNet Baseline 复现计划

## ✅ 已完成

1. **硬件分析** — CPU: i7-12700F, RAM: 32GB, GPU: RTX 3060, OS: Ubuntu 22.04
2. **sudo 免密码** — 已配置成功
3. **权限配置** — `~/.claude/settings.json` 已添加 `"allow": ["*"]`
4. **Docker** — 已安装 (v29.1.3)，服务已启动
5. **NVIDIA 驱动** — `nvidia-driver-595` 已安装
6. **Nouveau 已屏蔽** — 配置文件已写入，但格式有误需修复

## ⚠️ 重启前修复

当前 `/etc/modprobe.d/blacklist-nouveau.conf` 格式损坏（换行问题），重启后重启后修复：

```bash
# 重启后运行：
sudo tee /etc/modprobe.d/blacklist-nouveau.conf << 'EOF'
blacklist nouveau
options nouveau modeset=0
EOF
sudo update-initramfs -u -k all
```

---

## 🔄 重启后执行步骤

### Step 1: 验证 NVIDIA 驱动

```bash
nvidia-smi
# 应显示 RTX 3060
```

### Step 2: 启动 Docker

```bash
sudo systemctl start docker
```

### Step 3: 安装 nvidia-container-toolkit

```bash
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg

curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

sudo apt update
sudo apt install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

### Step 4: 测试 GPU 容器

```bash
docker run --rm --gpus all nvidia/cuda:11.4.0-base-ubuntu20.04 nvidia-smi
```

### Step 5: 构建项目 Docker 镜像

在项目根目录执行：

```bash
cd ~/graspnet-baseline
```

创建 `Dockerfile.20.04`:

```dockerfile
FROM nvidia/cuda:11.4.0-devel-ubuntu20.04

# 基础依赖
RUN apt update && apt install -y python3 python3-pip libgl1-mesa-glx libglib2.0-0 wget git && \
    ln -s /usr/bin/python3 /usr/bin/python

# 安装 PyTorch 1.11 + CUDA 11.3
RUN pip install torch==1.11.0+cu113 torchvision==0.12.0+cu113 --extra-index-url https://download.pytorch.org/whl/cu113

# 项目依赖
RUN pip install numpy scipy open3d pillow tqdm tensorboard scikit-learn

WORKDIR /workspace
COPY . .

# 编译 pointnet2
RUN cd pointnet2 && python setup.py install

# 编译 knn
RUN cd knn && python setup.py install

# 安装 graspnetAPI
RUN git clone https://github.com/graspnet/graspnetAPI.git && \
    cd graspnetAPI && pip install .

# 下载预训练 checkpoint
RUN mkdir -p logs/log_rs && \
    wget -O logs/log_rs/checkpoint.tar https://download.mygraspnet.net/checkpoint-rs.tar
```

构建镜像：

```bash
docker build -f Dockerfile.20.04 -t graspnet-baseline:latest .
```

### Step 6: 运行 Demo

```bash
docker run --rm --gpus all -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  graspnet-baseline:latest \
  python demo.py --checkpoint_path logs/log_rs/checkpoint.tar
```

> 如果无图形界面，修改 `demo.py` 将可视化改为保存截图：
> 将 `o3d.visualization.draw_geometries([cloud, *grippers])` 替换为：
> ```python
> vis = o3d.visualization.Visualizer()
> vis.create_window(visible=False)
> vis.add_geometry(cloud)
> for g in grippers:
>     vis.add_geometry(g)
> vis.poll_events()
> vis.capture_screen_image('demo_result.png')
> vis.destroy_window()
> ```

---

## 📁 关键路径

| 内容 | 路径 |
|------|------|
| 项目代码 | `<仓库根目录>/graspnet-baseline` |
| 本地工具配置 | 由开发者环境自行管理，不属于仓库 |
| Dockerfile | `graspnet-baseline/Dockerfile.20.04` |
| 预训练权重 | `logs/log_rs/checkpoint.tar` |
| 示例数据 | `doc/example_data/` |

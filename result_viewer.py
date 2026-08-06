#!/usr/bin/env python3
"""
result_viewer.py — 主机端 GraspNet 结果显示

监视共享目录中的 result.png，用 OpenCV 窗口实时显示。
运行在主机的 ROS 2 图形环境中（不需要 Docker）。
"""

import os
import sys
import time
import signal
import cv2
import numpy as np

SHARED_DIR = os.path.expanduser('~/piper_graspnet/shared_camera_data')
RESULT_PATH = os.path.join(SHARED_DIR, 'result.png')

running = True


def signal_handler(sig, frame):
    """SIGINT 处理：置 running=False 让主循环退出，干净关闭窗口。"""
    global running
    print("\nExiting...")
    running = False


def main():
    """主循环：轮询共享目录中的 result.png，检测到 mtime 变化就重新显示。

    为什么用 mtime 而非 .ready：本查看器只是"人眼观察"用途，
    不需要与推理进程严格握手——文件更新即刷新即可，简单可靠。
    result.png 由 Docker 内推理脚本的 display_loop 线程原子写入
    （先写 .tmp 再 rename），因此这里不会读到半帧。
    """
    global running
    signal.signal(signal.SIGINT, signal_handler)

    os.makedirs(SHARED_DIR, exist_ok=True)

    cv2.namedWindow("GraspNet - Perception Results", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("GraspNet - Perception Results", 1280, 720)

    last_mtime = 0
    print(f"Watching for results in {RESULT_PATH}")
    print("Press 'q' or Esc to exit")

    while running:
        try:
            if os.path.exists(RESULT_PATH):
                mtime = os.path.getmtime(RESULT_PATH)
                if mtime > last_mtime:
                    img = cv2.imread(RESULT_PATH)
                    if img is not None:
                        h, w = img.shape[:2]
                        # Resize to fit display while maintaining aspect ratio
                        max_w, max_h = 1280, 720
                        if w > max_w or h > max_h:
                            scale = min(max_w / w, max_h / h)
                            new_w, new_h = int(w * scale), int(h * scale)
                            img = cv2.resize(img, (new_w, new_h))
                        cv2.imshow("GraspNet - Perception Results", img)
                        last_mtime = mtime
        except Exception as e:
            # 单帧失败不中断轮询（如文件被原子替换的瞬间读取冲突）
            print(f"Error: {e}")

        key = cv2.waitKey(100) & 0xFF
        if key in (ord('q'), ord('Q'), 27):
            break

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()

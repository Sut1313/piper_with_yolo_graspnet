#!/usr/bin/env python3
"""Solve AgileX Piper IK with the ssik prebuilt analytic artifact.

用 ssik 预编译解析 IK 库求解 AgileX Piper 机械臂逆运动学。
此脚本是被 C++ 主节点（graspnet_6d_grasp_planner）以子进程方式调用的
命令行工具，替换 MoveIt KDL 求解失败的环节（KDL 对很多 6D 目标位姿
求不出精确解，ssik 解析解几乎总能给出候选）。

【命令行 API】
  --matrix  <16 个逗号分隔浮点数>  目标变换矩阵（行主序 4x4，link6 在
                                   base_link 下的位姿，同 Eigen 内存布局）
  --seed    <6 个逗号分隔浮点数>   可选的初始关节角种子（当前关节角），
                                   帮助解析器收敛到相近解
  --max-solutions <N>              最多返回的解个数（默认 8）
  --respect-limits                  是否限制关节角在运动范围内（默认开启）

【输出格式】
  每行一个解，空格分隔 7 个数：
    q0 q1 q2 q3 q4 q5 residual
  其中 q0..q5 是 6 个关节角（弧度），residual 是该解的正运动学残差
  （FK 误差，数量级 ~1e-10 表示高精度解）。
  C++ 侧逐行解析后按"距当前关节角最近"排序并逐个交给 MoveIt 做
  关节目标规划。

【退出码】
  0 成功（可能无解，无解时无输出）；2 ssik 导入失败；
  3 求解异常；4 --matrix 格式错误；5 --seed 格式错误。
"""

from __future__ import annotations

import argparse
import sys
import warnings

import numpy as np


def parse_args() -> argparse.Namespace:
    # 命令行参数定义。注意 --respect-limits 用 action="store_true"
    # 且 default=True：即默认开启限位，无法通过参数关闭（设计如此，
    # 因为越限关节解对执行无意义）。
    parser = argparse.ArgumentParser()
    parser.add_argument("--matrix", required=True)
    parser.add_argument("--seed")
    parser.add_argument("--max-solutions", type=int, default=8)
    parser.add_argument("--respect-limits", action="store_true", default=True)
    return parser.parse_args()


def parse_float_csv(value: str, expected_count: int, name: str) -> list[float]:
    """把逗号分隔的字符串解析成指定数量的浮点数列表。

    空项会被丢弃（容忍尾随逗号）；数量不匹配时抛 ValueError，
    由 main 捕获后以对应退出码报错。
    """
    parts = [part for part in value.split(",") if part]
    if len(parts) != expected_count:
        raise ValueError(f"{name} expected {expected_count} values, got {len(parts)}")
    return [float(part) for part in parts]


def main() -> int:
    args = parse_args()
    # 屏蔽 ssik 内部大量告警（解析求解器常打印收敛性警告），
    # 保持 stdout 只有"每行一个解"的干净输出，方便 C++ 侧解析。
    warnings.filterwarnings("ignore")

    # 延迟导入 ssik：只有真正执行时才加载这个重型解析库，
    # 避免环境未安装 ssik 时脚本在 import 阶段就崩掉。
    try:
        from ssik.prebuilt.agilex import piper_ik
    except Exception as exc:
        print(f"ERROR import ssik Piper IK failed: {exc}", file=sys.stderr)
        return 2

    # 解析 4x4 目标位姿（行主序 16 个元素）→ 4x4 ndarray
    try:
        target_values = parse_float_csv(args.matrix, 16, "matrix")
        target = np.array(target_values, dtype=np.float64).reshape((4, 4))
    except Exception as exc:
        print(f"ERROR invalid matrix: {exc}", file=sys.stderr)
        return 4

    # 可选的种子关节角：传入当前关节角可让求解器优先返回附近的解
    # （对"从当前位置平滑过渡"很重要）。
    seed = None
    if args.seed is not None:
        try:
            seed = np.array(parse_float_csv(args.seed, 6, "seed"), dtype=np.float64)
        except Exception as exc:
            print(f"ERROR invalid seed: {exc}", file=sys.stderr)
            return 5

    # 调用 ssik 解析 IK：
    #   allow_refinement：对解做牛顿迭代精化（提升 FK 精度）
    #   allow_rescue：主解析流程失败时用数值法兜底，尽量保证有解
    #   max_solutions 至少为 1，避免传入 0 或负数
    try:
        solutions = piper_ik.solve(
            target,
            max_solutions=max(1, args.max_solutions),
            q_seed=seed,
            respect_limits=args.respect_limits,
            allow_refinement=True,
            allow_rescue=True,
        )
    except Exception as exc:
        print(f"ERROR solve failed: {exc}", file=sys.stderr)
        return 3

    # 输出：每行 6 个关节角 + 1 个 FK 残差，精确到 12 位有效数字，
    # 供 C++ 子进程解析（fscanf 空格分隔）。
    for solution in solutions:
        q = np.asarray(solution.q, dtype=np.float64)
        residual = float(solution.fk_residual)
        values = [*q.tolist(), residual]
        print(" ".join(f"{value:.12g}" for value in values))

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

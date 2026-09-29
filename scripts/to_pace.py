#!/usr/bin/env python3
"""
把 pace_chirp 的一次采集(ticks.csv + meta.json)转换为 pace-sim2real 的 chirp_data.pt。

用法:
    python3 to_pace.py <运行目录> --dt <PACE 仿真 physics dt> [--joint-order a,b,...] [-o chirp_data.pt]

输出格式与 pace-sim2real scripts/pace/data_collection.py 保存的一致:
    {"time": (N,), "dof_pos": (N, J), "des_dof_pos": (N, J)}, float32, URDF 坐标, 按 joint_order 排列
另生成 <输出名>_info.json, 记录关节顺序、kp/kd、仿真中需要复现的保持/无力关节等信息。

重采样:
    des_dof_pos: 每个网格时刻取 t_cmd 不晚于它的最后一条指令(零阶保持), 即电机当时正在执行的指令
    dof_pos    : 对回传帧 (rx_ns, q) 线性插值; --meas zoh 时取不晚于网格时刻的最后一帧
                 (zoh 会额外引入平均约半个回传周期的滞后, 被 PACE 算进延迟)
时间窗口默认从 hold_pre 开始(关节静止在中心位置, 与仿真初始状态一致)到 hold_post 结束。
踝(并联): 默认用 motor_q_<m> 列按严格收敛重新正解 pitch/roll(见 ankle_fk.py), 消除底层 fk 收敛阈值
          造成的台阶误差; --ankle-fk lib 则直接使用采集时记录的 q(与部署时底层解算一致)。
"""
import argparse
import csv
import json
import os
import sys

import numpy as np

from ankle_fk import ANKLES, motor_to_joint

NS = 1e-9


def load_ticks(path):
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
        fields = reader.fieldnames
    cols = {}
    for key in fields:
        if key == "phase":
            cols[key] = np.array([r[key] for r in rows])
        else:
            cols[key] = np.array([float(r[key]) if r[key] != "" else np.nan for r in rows])
    return cols


def phase_window(phase, t, start_phase, end_phase):
    start_idx = np.where(phase == start_phase)[0]
    end_idx = np.where(phase == end_phase)[0]
    if len(start_idx) == 0 or len(end_idx) == 0:
        sys.exit(f"采集中没有 {start_phase} 或 {end_phase} 阶段(采集是否中止了?)")
    return t[start_idx[0]], t[end_idx[-1]]


def main():
    parser = argparse.ArgumentParser(description="pace_chirp 采集 -> PACE chirp_data.pt")
    parser.add_argument("run_dir", help="pace_chirp 输出目录")
    parser.add_argument("--dt", type=float, required=True, help="PACE 仿真的 physics dt [s], 数据按此步长重采样")
    parser.add_argument("--joint-order", help="逗号分隔的关节名, 必须与 PACE 环境 sim2real.joint_order 一致; 默认为采集中全部参与关节(按下标)")
    parser.add_argument("--start", default="hold_pre", help="起始阶段, 默认 hold_pre")
    parser.add_argument("--end", default="hold_post", help="结束阶段, 默认 hold_post")
    parser.add_argument("--meas", choices=["linear", "zoh"], default="linear", help="测量值重采样方式")
    parser.add_argument("--ankle-fk", choices=["exact", "lib"], default="exact",
                        help="踝关节角: exact 用电机原始角严格正解(默认), lib 用采集时底层解算的值")
    parser.add_argument("-o", "--output", help="输出文件, 默认 <运行目录>/chirp_data.pt")
    args = parser.parse_args()

    meta = json.load(open(os.path.join(args.run_dir, "meta.json"), encoding="utf-8"))
    if meta.get("schema") != "pace_chirp/1":
        sys.exit(f"不支持的 meta schema: {meta.get('schema')}(需要 pace_chirp 的输出)")
    if not meta["completed"]:
        print(f"警告: 该次采集未完成, 原因: {meta['abort_reason']}", file=sys.stderr)
    d = load_ticks(os.path.join(args.run_dir, "ticks.csv"))
    joints = {j["name"]: j for j in meta["joints"]}

    if args.joint_order:
        order = [n.strip() for n in args.joint_order.split(",") if n.strip()]
        missing = [n for n in order if n not in joints]
        if missing:
            sys.exit(f"这些关节不在本次采集中: {missing}")
    else:
        order = [j["name"] for j in meta["joints"]]

    ankle_fk_used = {}
    if args.ankle_fk == "exact":
        for ip, (side, ma, mb, ir) in ANKLES.items():
            if f"q_{ip}" not in d:
                continue
            cols = [f"motor_q_{ma}", f"motor_q_{mb}", f"motor_rx_count_{ma}", f"motor_rx_count_{mb}"]
            if any(c not in d for c in cols):
                print(f"警告: 缺少 {ip}/{ir} 的电机原始列(旧版 pace_chirp 采集), 踝使用采集时记录的 q", file=sys.stderr)
                continue
            # 两个电机都已有回传的行才能解算
            ok = (d[f"motor_rx_count_{ma}"] > 0) & (d[f"motor_rx_count_{mb}"] > 0)
            p, r = motor_to_joint(side, d[f"motor_q_{ma}"][ok], d[f"motor_q_{mb}"][ok], d[f"q_{ip}"][ok], d[f"q_{ir}"][ok])
            if not (np.all(np.isfinite(p)) and np.all(np.isfinite(r))):
                sys.exit(f"踝 {ip}/{ir} 严格正解出现 NaN/Inf, 可改用 --ankle-fk lib")
            dp = p - d[f"q_{ip}"][ok]
            dr = r - d[f"q_{ir}"][ok]
            d[f"q_{ip}"] = d[f"q_{ip}"].copy()
            d[f"q_{ir}"] = d[f"q_{ir}"].copy()
            d[f"q_{ip}"][ok] = p
            d[f"q_{ir}"][ok] = r
            for i, e in ((ip, dp), (ir, dr)):
                ankle_fk_used[i] = {"rms_change_rad": float(np.sqrt(np.mean(e ** 2))), "max_change_rad": float(np.abs(e).max())}
                print(f"踝 {i}: 严格正解相对采集记录值 均方根 {ankle_fk_used[i]['rms_change_rad'] * 1e3:.2f} mrad, "
                      f"最大 {ankle_fk_used[i]['max_change_rad'] * 1e3:.2f} mrad")

    t_cmd = d["t_cmd_ns"]
    t_begin, t_end = phase_window(d["phase"], t_cmd, args.start, args.end)
    n = int(np.floor((t_end - t_begin) * NS / args.dt)) + 1
    grid = t_begin + np.arange(n) * args.dt / NS

    des = np.zeros((n, len(order)), dtype=np.float32)
    pos = np.zeros((n, len(order)), dtype=np.float32)
    for c, name in enumerate(order):
        i = joints[name]["index"]
        # 零阶保持: 网格时刻之前最后一条指令
        k = np.searchsorted(t_cmd, grid, side="right") - 1
        des[:, c] = d[f"q_des_{i}"][k]
        # 回传帧: rx_count 增加的行
        rc = d[f"rx_count_{i}"]
        fresh = np.zeros(len(rc), dtype=bool)
        fresh[1:] = np.diff(rc) > 0
        rx_t = d[f"rx_ns_{i}"][fresh]
        rx_q = d[f"q_{i}"][fresh]
        if rx_t[0] > grid[0] or rx_t[-1] < grid[-1]:
            sys.exit(f"{name} 的回传帧没有覆盖整个时间窗口")
        if args.meas == "linear":
            pos[:, c] = np.interp(grid, rx_t, rx_q)
        else:
            pos[:, c] = rx_q[np.searchsorted(rx_t, grid, side="right") - 1]

    time = (np.arange(n) * args.dt).astype(np.float32)
    out = args.output or os.path.join(args.run_dir, "chirp_data.pt")
    try:
        import torch
        torch.save({
            "time": torch.from_numpy(time),
            "dof_pos": torch.from_numpy(pos),
            "des_dof_pos": torch.from_numpy(des),
        }, out)
    except ImportError:
        out = os.path.splitext(out)[0] + ".npz"
        np.savez(out, time=time, dof_pos=pos, des_dof_pos=des)
        print(f"警告: 未安装 torch, 改存为 {out}(键名相同), 需在 Isaac Lab 环境中再转成 .pt", file=sys.stderr)

    in_order = set(order)
    info = {
        "source_run": os.path.abspath(args.run_dir),
        "dt": args.dt,
        "num_steps": n,
        "window": [args.start, args.end],
        "meas_resample": args.meas,
        "angle_frame": meta["angle_frame"],
        "joint_order": order,
        "gains": {name: {"kp": joints[name]["kp"], "kd": joints[name]["kd"]} for name in order},
        "initial_pos": {name: float(des[0, c]) for c, name in enumerate(order)},
        # 仿真中必须按同样方式复现的其余关节
        "held_not_in_order": {j["name"]: {"pos": j["q0"], "kp": j["kp"], "kd": j["kd"]}
                              for j in meta["joints"] if j["name"] not in in_order},
        "limp_joints": meta["limp_parallel_joints"],
        # 踝(并联): 真机上是关节空间 PD(与仿真一致), 另有电机侧阻尼 kd_ff 与力矩限幅, 仿真中如需复现见此处
        "ankle": {
            "joints": [j["name"] for j in meta["joints"] if j.get("parallel")],
            **meta.get("ankle", {}),
            "dof_pos_fk": {"method": "exact" if ankle_fk_used else "lib",
                           "change_vs_recorded": {meta_name: ankle_fk_used[j["index"]]
                                                  for j in meta["joints"] for meta_name in [j["name"]]
                                                  if j["index"] in ankle_fk_used}},
        },
        "chirp": meta["chirp"],
    }
    info_path = os.path.splitext(out)[0] + "_info.json"
    with open(info_path, "w", encoding="utf-8") as f:
        json.dump(info, f, indent=2, ensure_ascii=False)

    print(f"已写入 {out}: {n} 步 x {len(order)} 关节, dt={args.dt}, 时长 {time[-1]:.2f} s")
    print(f"已写入 {info_path}")
    print("关节顺序:", ", ".join(order))
    if info["held_not_in_order"]:
        print("注意: 以下关节在真机上被 PD 保持但不在 joint_order 中, 仿真里需保持在相同位置:",
              ", ".join(info["held_not_in_order"]))
    if info["ankle"]["joints"]:
        print(f"注意: 踝为并联关节, 真机上关节空间 PD + 电机侧阻尼 kd_ff={info['ankle'].get('kd_ff', 0)}, "
              "关节力矩限幅 pitch 60 / roll 20 Nm, 电机力矩限幅 25 Nm")
    if info["limp_joints"]:
        print("注意: 以下并联关节在真机上无力, 仿真里需设为零刚度:", ", ".join(info["limp_joints"]))


if __name__ == "__main__":
    main()

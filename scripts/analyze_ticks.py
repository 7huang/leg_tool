#!/usr/bin/env python3
"""
分析 pace_single_joint / pace_chirp 的输出(ticks.csv + meta.json)。

用法:
    python3 analyze_ticks.py <运行目录 或 ticks.csv> [--joint 下标] [--urdf robot.urdf] [--out 输出目录] [--show]

对每个被激励关节(pace_chirp 可用 --joint 只看其中一个):
    1. 拆分指令序列 (t_cmd_ns, q_des) 与测量序列 (rx_count 增加的行: rx_ns, q, qd, tau)
    2. 发送/回传间隔统计, 判断丢帧发生在发送端还是回传端; 发送->回传延迟
    3. chirp 段闭环频率响应 H = q / q_des, 并拟合
       I*qdd + (kd+b)*qd + Fc*sign(qd) = kp*(q_des(t-T) - q)
       得到惯量 I、粘滞阻尼 b、库伦摩擦 Fc、延迟 T 的粗估计(仅用于检查数据, 正式辨识交给 PACE)
另打印一次控制循环时序抖动。

输出: 终端统计 + 每个关节的 analysis_time_<i>.png / analysis_timing_<i>.png / analysis_bode_<i>.png
只依赖 numpy 和 matplotlib。
"""
import argparse
import csv
import json
import os
import sys

import numpy as np

NS = 1e-9
ACTIVE_PHASES = ("engage", "move_pose", "settle", "move_in", "hold_pre", "chirp", "hold_post", "move_out", "move_home",
                 "release")
JOINT_FIELDS = ("q_des", "kp", "kd", "q", "qd", "tau", "rx_ns", "rx_count", "tx_ns", "tx_count")


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


def joint_views(d, meta):
    """把两种输出格式统一成逐关节视图: 名称、增益与各列数据"""
    if meta["schema"] == "pace_single_joint/1":
        return [{
            "index": meta["joint_index"], "name": meta.get("joint_name") or f"joint {meta['joint_index']}",
            "motor_id": meta["motor_id"], "kp": meta["kp"], "kd": meta["kd"], "excited": True,
            "cols": {f: d[f] for f in JOINT_FIELDS},
        }]
    if meta["schema"] == "pace_chirp/1":
        return [{
            "index": j["index"], "name": j["name"], "motor_id": j["motor_id"], "kp": j["kp"], "kd": j["kd"],
            "excited": j["role"] == "excite",
            "cols": {f: d[f"{f}_{j['index']}"] for f in JOINT_FIELDS},
        } for j in meta["joints"]]
    sys.exit(f"不支持的 meta schema: {meta.get('schema')}")


def new_event_mask(count):
    """count 比上一行增加的行 = 一个新事件(新回传帧 / 新发送帧)"""
    mask = np.zeros(len(count), dtype=bool)
    mask[0] = count[0] > 0
    mask[1:] = np.diff(count) > 0
    return mask


def describe_ms(name, x_ms):
    if len(x_ms) == 0:
        print(f"  {name}: 无数据")
        return
    p50, p99 = np.percentile(x_ms, [50, 99])
    print(f"  {name}: n={len(x_ms)} 均值={x_ms.mean():.3f} 中位={p50:.3f} p99={p99:.3f} 最大={x_ms.max():.3f} ms")


def slot_histogram(name, interval_ms, slot_ms=4.0):
    """按发送轮询周期(4ms)的整数倍统计间隔, k=1 为正常, k>=2 表示中间错过 k-1 个发送机会"""
    k = np.rint(interval_ms / slot_ms).astype(int)
    values, counts = np.unique(k, return_counts=True)
    text = ", ".join(f"{v * slot_ms:.0f}ms:{c}" for v, c in zip(values, counts))
    print(f"  {name} 按 {slot_ms:.0f}ms 取整分布: {text}")


def chirp_phase(t, f0, f1, duration):
    return 2 * np.pi * (f0 * t + 0.5 * (f1 - f0) / duration * t * t)


def chirp_freq(t, f0, f1, duration):
    return f0 + (f1 - f0) * t / duration


def fit_sine(t, y, phi, tc):
    """y ~ a*sin(phi) + b*cos(phi) + c + d*(t-tc), 返回复幅值 a + i*b"""
    X = np.column_stack([np.sin(phi), np.cos(phi), np.ones_like(t), t - tc])
    coef, *_ = np.linalg.lstsq(X, y, rcond=None)
    return coef[0] + 1j * coef[1]


def frequency_response(meas_t, meas_q, cmd_t, cmd_q, chirp):
    """在 chirp 包络为 1 的区间滑窗, 用已知 chirp 相位拟合指令与测量的复幅值, H = Y / U"""
    f0, f1, T, ramp = chirp["f0"], chirp["f1"], chirp["duration"], chirp["ramp"]
    freqs, H, motion_amp = [], [], []
    t = ramp
    while True:
        w = float(np.clip(1.0 / chirp_freq(t, f0, f1, T), 0.5, 3.0))  # 约 1 个周期
        if t + w > T - ramp:
            break
        tc = t + w / 2
        mm = (meas_t >= t) & (meas_t < t + w)
        mc = (cmd_t >= t) & (cmd_t < t + w)
        if mm.sum() > 20 and mc.sum() > 20:
            Y = fit_sine(meas_t[mm], meas_q[mm], chirp_phase(meas_t[mm], f0, f1, T), tc)
            U = fit_sine(cmd_t[mc], cmd_q[mc], chirp_phase(cmd_t[mc], f0, f1, T), tc)
            freqs.append(chirp_freq(tc, f0, f1, T))
            H.append(Y / U)
            motion_amp.append(abs(Y))
        t += w / 2
    return np.array(freqs), np.array(H), np.array(motion_amp)


def fit_actuator_model(freqs, H, motion_amp, kp, kd, kg=0.0, max_delay_ms=30.0):
    """
    闭环模型: H (-I w^2 + i w (kd + b) + i 4Fc/(pi X) + kp + kg) = kp e^{-i w T}
    库伦摩擦用描述函数等效(X 为运动幅值); kg 为重力刚度(由 URDF 给定, 不参与拟合)。
    对每个 T 网格求 I, b, Fc 的线性最小二乘, 取残差最小者。
    """
    w = 2 * np.pi * freqs
    best = None
    for T in np.arange(0.0, max_delay_ms * 1e-3 + 1e-9, 1e-4):
        A = np.column_stack([-w ** 2 * H, 1j * w * H, 1j * 4 / (np.pi * motion_amp) * H])
        rhs = kp * np.exp(-1j * w * T) - (kp + kg) * H - 1j * w * kd * H
        A_ri = np.vstack([A.real, A.imag])
        rhs_ri = np.concatenate([rhs.real, rhs.imag])
        coef, *_ = np.linalg.lstsq(A_ri, rhs_ri, rcond=None)
        res = np.sum((A_ri @ coef - rhs_ri) ** 2)
        if best is None or res < best[0]:
            best = (res, T, coef)
    _, T, (I, b, Fc) = best
    return {"I": I, "b": b, "Fc": Fc, "T": T, "kg": kg}


def model_response(freqs, motion_amp, kp, kd, p):
    w = 2 * np.pi * freqs
    den = -p["I"] * w ** 2 + 1j * w * (kd + p["b"]) + 1j * 4 * p["Fc"] / (np.pi * motion_amp) + kp + p.get("kg", 0.0)
    return kp * np.exp(-1j * w * p["T"]) / den


def joint_pose(meta):
    """采集时的关节姿态(各关节中心位置), 用于 URDF 计算"""
    if meta["schema"] == "pace_single_joint/1":
        return {meta.get("joint_name", ""): meta["q0"]}
    return {j["name"]: j["center"] for j in meta["joints"]}


def analyze_joint(jv, d, meta, out_dir, plt, show, robot=None):
    c = jv["cols"]
    phase = d["phase"]
    kp, kd = jv["kp"], jv["kd"]
    chirp = meta["chirp"]
    suffix = f"_{jv['index']}"
    print(f"\n==== {jv['name']} (index {jv['index']}, motor {jv['motor_id']}), kp={kp} kd={kd}")

    # ---------- 1. 拆分序列 ----------
    rx_mask = new_event_mask(c["rx_count"])
    tx_mask = new_event_mask(c["tx_count"])
    meas = {k: c[k][rx_mask] for k in ("rx_ns", "q", "qd", "tau")}
    meas_phase = phase[rx_mask]
    tx_ns = c["tx_ns"][tx_mask]
    rx_jump = np.diff(c["rx_count"])
    multi = int(np.sum(rx_jump[rx_jump > 1] - 1))
    print("[1] 序列")
    print(f"  指令: {len(d['t_cmd_ns'])} 条 (每拍一条)")
    print(f"  测量: {rx_mask.sum()} 帧, 回传计数总增量 {int(c['rx_count'][-1])}; 两次读取之间到达多帧而未记录的: {multi}")
    print(f"  发送: {tx_mask.sum()} 帧, 发送计数总增量 {int(c['tx_count'][-1])}")

    # ---------- 2. 发送 / 回传时序 ----------
    print("[2] 发送 / 回传时序 (chirp 段)")
    chirp_tick = d["t_tick_ns"][phase == "chirp"]
    t_c0, t_c1 = chirp_tick[0], chirp_tick[-1]
    chirp_rx = meas["rx_ns"][meas_phase == "chirp"]
    chirp_tx = tx_ns[(tx_ns >= t_c0) & (tx_ns <= t_c1)]
    rx_int = np.diff(chirp_rx) * 1e-6
    tx_int = np.diff(chirp_tx) * 1e-6
    describe_ms("回传间隔", rx_int)
    slot_histogram("回传间隔", rx_int)
    describe_ms("发送间隔", tx_int)
    slot_histogram("发送间隔", tx_int)

    # 每个回传帧对应它之前最近的一次发送
    idx = np.searchsorted(tx_ns, meas["rx_ns"], side="right") - 1
    valid = idx >= 0
    tx_to_rx = (meas["rx_ns"][valid] - tx_ns[idx[valid]]) * 1e-6
    describe_ms("发送->回传解包", tx_to_rx)

    # 回传间隔 > 6ms 时, 看这段时间里发出了几帧: <=1 说明是发送端没发, >=2 说明回传丢了
    gap_idx = np.where(rx_int > 6.0)[0]
    tx_side = reply_side = 0
    for g in gap_idx:
        n_tx = np.sum((chirp_tx > chirp_rx[g]) & (chirp_tx < chirp_rx[g + 1]))
        if n_tx <= 1:
            tx_side += 1
        else:
            reply_side += 1
    print(f"  回传间隔>6ms 共 {len(gap_idx)} 次: 发送端未发 {tx_side} 次, 已发送但回传缺失 {reply_side} 次")

    # ---------- 3. 频率响应 ----------
    cmd_sel = phase == "chirp"
    cmd_t = (d["t_cmd_ns"][cmd_sel] - t_c0) * NS
    cmd_q = c["q_des"][cmd_sel]
    meas_sel = (meas["rx_ns"] >= t_c0) & (meas["rx_ns"] <= t_c1)
    meas_t = (meas["rx_ns"][meas_sel] - t_c0) * NS
    meas_q = meas["q"][meas_sel]
    params = None
    if jv["excited"]:
        print("[3] chirp 段闭环频率响应")
        freqs, H, motion_amp = frequency_response(meas_t, meas_q, cmd_t, cmd_q, chirp)
        for f, h in zip(freqs[:: max(1, len(freqs) // 12)], H[:: max(1, len(freqs) // 12)]):
            print(f"  f={f:5.2f} Hz  |H|={abs(h):.3f}  相位={np.degrees(np.angle(h)):7.1f} deg")
        urdf = robot.joint_terms(jv["name"], joint_pose(meta)) if robot is not None and jv["name"] in robot.joints else None
        params = fit_actuator_model(freqs, H, motion_amp, kp, kd, kg=urdf["k_g"] if urdf else 0.0)
        H_model = model_response(freqs, motion_amp, kp, kd, params)
        rel_err = np.abs(H_model - H) / np.abs(H)
        print(f"  模型粗估: I={params['I']:.4f} kg*m^2  b={params['b']:.3f} Nm*s/rad  "
              f"Fc={params['Fc']:.3f} Nm  T={params['T'] * 1e3:.1f} ms  (复响应相对误差 中位 {np.median(rel_err) * 100:.1f}%)")
        if urdf:
            armature = params["I"] - urdf["I_link"]
            print(f"  URDF: 重力刚度 k_g={urdf['k_g']:.2f} Nm/rad(已计入模型), 连杆惯量 {urdf['I_link']:.4f} kg*m^2 "
                  f"=> armature 约 {armature:.4f} kg*m^2{'  (为负: 数据不可信或基座未固定)' if armature < 0 else ''}")
    else:
        print(f"[3] 保持关节: chirp 段 |q - q_des| 最大 {np.max(np.abs(meas_q - np.interp(meas_t, cmd_t, cmd_q))):.4f} rad")

    if plt is None:
        return
    import matplotlib

    # ---------- 图 ----------
    t0 = d["t_tick_ns"][0]
    tc_all = (d["t_cmd_ns"] - t0) * NS
    tm_all = (meas["rx_ns"] - t0) * NS
    q_des_at_rx = np.interp(meas["rx_ns"], d["t_cmd_ns"], c["q_des"])
    kp_at_rx = np.interp(meas["rx_ns"], d["t_cmd_ns"], c["kp"])
    kd_at_rx = np.interp(meas["rx_ns"], d["t_cmd_ns"], c["kd"])
    active = np.isin(meas_phase, ACTIVE_PHASES)

    fig, ax = plt.subplots(4, 1, figsize=(12, 11))
    ax[0].plot(tc_all, c["q_des"], lw=0.8, label="q_des (t_cmd)")
    ax[0].plot(tm_all, meas["q"], lw=0.8, label="q (rx)")
    ax[0].set_ylabel("rad")
    ax[0].set_title(f"{jv['name']} 全程")
    ax[0].legend(loc="upper right")
    z0, z1 = (t_c1 - t0) * NS - 2.0, (t_c1 - t0) * NS
    zc = (tc_all >= z0) & (tc_all <= z1)
    zm = (tm_all >= z0) & (tm_all <= z1)
    ax[1].plot(tc_all[zc], c["q_des"][zc], lw=1.0, label="q_des")
    ax[1].plot(tm_all[zm], meas["q"][zm], ".", ms=3, label="q 回传帧")
    ax[1].set_ylabel("rad")
    ax[1].set_title("chirp 最后 2 s (最高频率段)")
    ax[1].legend(loc="upper right")
    ax[2].plot(tm_all[active], (meas["q"] - q_des_at_rx)[active], lw=0.8)
    ax[2].set_ylabel("q - q_des [rad]")
    ax[2].set_title("跟踪误差(出力阶段)")
    ax[3].plot(tm_all[active], meas["tau"][active], lw=0.8, label="tau 回传")
    ax[3].plot(tm_all[active], (kp_at_rx * (q_des_at_rx - meas["q"]) - kd_at_rx * meas["qd"])[active], lw=0.8, alpha=0.7,
               label="kp*(q_des-q) - kd*qd")
    ax[3].set_ylabel("Nm")
    ax[3].set_xlabel("t [s]")
    ax[3].legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, f"analysis_time{suffix}.png"), dpi=110)

    fig, ax = plt.subplots(2, 2, figsize=(12, 7))
    ax[0, 0].hist(rx_int, bins=np.arange(0, max(rx_int.max(), 20) + 0.5, 0.25))
    ax[0, 0].set_title("回传间隔 [ms] (chirp)")
    ax[0, 0].set_yscale("log")
    ax[0, 1].hist(tx_int, bins=np.arange(0, max(tx_int.max(), 20) + 0.5, 0.25))
    ax[0, 1].set_title("发送间隔 [ms] (chirp)")
    ax[0, 1].set_yscale("log")
    ax[1, 0].hist(tx_to_rx, bins=100)
    ax[1, 0].set_title("发送 -> 回传解包 [ms]")
    ax[1, 1].hist((d["t_cmd_ns"] - d["t_tick_ns"]) * 1e-6, bins=100)
    ax[1, 1].set_title("t_cmd - t_tick [ms]")
    ax[1, 1].set_yscale("log")
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, f"analysis_timing{suffix}.png"), dpi=110)

    if params is not None:
        fig, ax = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
        ax[0].semilogx(freqs, 20 * np.log10(np.abs(H)), "o", label="实测")
        ax[0].semilogx(freqs, 20 * np.log10(np.abs(H_model)), "-", label="模型粗估")
        ax[0].set_ylabel("|q/q_des| [dB]")
        ax[0].legend()
        ax[0].set_title(f"{jv['name']}  I={params['I']:.4f}  b={params['b']:.3f}  Fc={params['Fc']:.3f}  "
                        f"T={params['T'] * 1e3:.1f}ms")
        ax[1].semilogx(freqs, np.degrees(np.angle(H)), "o")
        ax[1].semilogx(freqs, np.degrees(np.angle(H_model)), "-")
        ax[1].set_ylabel("相位 [deg]")
        ax[1].set_xlabel("f [Hz]")
        # 对数轴默认用 10^{-1} 形式, 中文字体缺数学负号, 改为普通小数
        tick_fmt = matplotlib.ticker.FuncFormatter(lambda v, _: f"{v:g}")
        for a in ax:
            a.grid(True, which="both", alpha=0.3)
            a.xaxis.set_major_formatter(tick_fmt)
            a.xaxis.set_minor_formatter(tick_fmt)
        fig.tight_layout()
        fig.savefig(os.path.join(out_dir, f"analysis_bode{suffix}.png"), dpi=110)
    if not show:
        plt.close("all")  # 多关节时避免图窗累积占用内存


def main():
    parser = argparse.ArgumentParser(description="分析 pace_single_joint / pace_chirp 输出")
    parser.add_argument("path", help="运行目录或 ticks.csv")
    parser.add_argument("--joint", type=int, help="只分析该下标的关节(默认分析全部被激励关节)")
    parser.add_argument("--all", action="store_true", help="同时分析保持关节")
    parser.add_argument("--out", help="图片输出目录, 默认与 ticks.csv 相同")
    parser.add_argument("--no-plot", action="store_true", help="不生成图片")
    parser.add_argument("--show", action="store_true", help="弹出图窗")
    parser.add_argument("--urdf", help="机器人 URDF: 模型中计入重力刚度, 并给出连杆惯量与 armature 估计")
    args = parser.parse_args()

    ticks_path = os.path.join(args.path, "ticks.csv") if os.path.isdir(args.path) else args.path
    run_dir = os.path.dirname(os.path.abspath(ticks_path))
    meta_path = os.path.join(run_dir, "meta.json")
    if not os.path.exists(meta_path):
        sys.exit(f"找不到 {meta_path}")
    meta = json.load(open(meta_path, encoding="utf-8"))
    out_dir = args.out or run_dir
    os.makedirs(out_dir, exist_ok=True)

    d = load_ticks(ticks_path)
    print(f"== {meta['schema']}, completed={meta['completed']} {meta.get('abort_reason', '')}")
    if "chirp" not in set(d["phase"]):
        sys.exit("没有 chirp 阶段的数据(采集是否在 chirp 之前中止了?)")

    print("\n[控制循环]")
    describe_ms("t_read - t_tick", (d["t_read_ns"] - d["t_tick_ns"]) * 1e-6)
    describe_ms("t_cmd - t_tick", (d["t_cmd_ns"] - d["t_tick_ns"]) * 1e-6)
    describe_ms("拍间隔", np.diff(d["t_tick_ns"]) * 1e-6)

    plt = None
    if not args.no_plot:
        import matplotlib
        if not args.show:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt

    views = joint_views(d, meta)
    if args.joint is not None:
        views = [v for v in views if v["index"] == args.joint]
        if not views:
            sys.exit(f"本次采集中没有关节 {args.joint}")
    elif not args.all:
        views = [v for v in views if v["excited"]]
    robot = None
    if args.urdf:
        from urdf_dynamics import Robot
        robot = Robot(args.urdf)
    for jv in views:
        analyze_joint(jv, d, meta, out_dir, plt, args.show, robot)

    if plt is not None:
        print(f"\n图已保存到 {out_dir}: analysis_time_<i>.png, analysis_timing_<i>.png, analysis_bode_<i>.png")
        if args.show:
            plt.show()


if __name__ == "__main__":
    try:
        import matplotlib
        matplotlib.rcParams["font.sans-serif"] = ["SimHei", "Noto Sans CJK SC", "WenQuanYi Micro Hei", "DejaVu Sans"]
        matplotlib.rcParams["axes.unicode_minus"] = False
    except ImportError:
        pass
    main()

# 说明
本项目是QL双足电机测试工具vDD-preview。本工具不依赖ROS或ROS2。

# 快速使用
在项目目录下
mkdir build
cd build
cmake ..
make
sudo su

./leg_tool
然后根据提示使用功能。

# PACE 单关节采集(pace_single_joint)
用于 PACE 真机数据采集的最小实验: 控制单个关节做 chirp 扫频运动, 同时记录每一拍的下发指令和每一帧电机回传, 所有时间戳使用同一时钟(CLOCK_MONOTONIC)。

与 leg_tool 一起编译, 编译后在 build 目录下生成 `pace_single_joint`。

## 运行前准备
1. 关闭自启动服务, 并退出 leg_tool / deploy_real。三者共用一把 CAN 锁(/tmp/leg_can_owner.lock), 不能同时运行。
```
systemctl stop keenon_humanoid.service
```
2. 用 leg_tool 的 666 检查角度正常, 用 2345 读出被测关节的 PVT 参数并保存。
3. 机器人躯干刚性吊装, 被测关节活动范围内无障碍, 实体急停可触达。
4. 让被测肢体自然下垂、静止。程序启动时处于零刚度, 若关节在动会拒绝继续。

## 使用
先用 --dry-run 检查波形(不连接电机, 只生成 plan.csv):
```
sudo ./pace_single_joint --joint 18 --name left_elbow --kp 20 --kd 1 --dry-run
```
真机运行(必须带 --confirm-suspended):
```
sudo ./pace_single_joint --joint 18 --name left_elbow --kp 20 --kd 1 --amp 0.05 --f0 0.1 --f1 2 --duration 20 --confirm-suspended
```
首次运行建议小振幅(0.05 rad)、低频(f1 <= 2 Hz), 确认数据正常后再逐步加大。

| 参数 | 说明 | 默认值 |
|---|---|---|
| --joint | 关节下标 0-28(MotorCmd 数组下标, 不是电机 ID), 不支持 4/5/10/11/13/14 | 必填 |
| --kp / --kd | PD 增益, 必须与仿真/部署一致 | 必填 |
| --name | 关节名, 只写入 meta.json | 空 |
| --amp | chirp 振幅 [rad], 上限 0.3 | 0.05 |
| --f0 / --f1 | 起止频率 [Hz], 上限 10 | 0.1 / 2.0 |
| --duration | chirp 时长 [s] | 20 |
| --ramp | chirp 首尾余弦淡入淡出时长 [s] | 1.0 |
| --hold-pre / --hold-post | chirp 前/后保持时长 [s] | 1.0 / 1.0 |
| --rate | 控制/记录频率 [Hz], 100-1000 | 1000 |
| --max-vel | 速度安全阈值 [rad/s] | 5 |
| --max-tau | 力矩安全阈值 [Nm] | 10 |
| --pos-margin | 位置超出 q0 +/- (amp + margin) 即中止 [rad] | 0.1 |
| --stale-ms | 反馈超过该时长未更新即中止 [ms] | 50 |
| --out | 输出根目录 | ./pace_log |
| --dry-run | 只生成 plan.csv, 不连接电机 | 关 |
| --confirm-suspended | 确认已吊装、急停可用, 真机运行必需 | 关 |

参考峰值速度 amp * 2π * f1 超过 0.8 * max-vel 时程序拒绝运行。

## 运行过程
程序自动依次执行:
1. probe: 零刚度 + 阻尼, 读取关节当前位置作为 q0(要求关节静止)
2. engage: 0.5 s 内 kp 平滑升到目标值
3. hold_pre: 保持 q0
4. chirp: q_des = q0 + 线性扫频, 首尾淡入淡出
5. hold_post: 保持 q0
6. release: 0.5 s 内 kp 平滑降到 0
7. zero: 发零力矩指令, 之后停止向该电机发送报文

以下任一情况立即中止, 进入 damp(kp = 0, 只保留阻尼), 然后照常把数据写入文件:
- 反馈超过 --stale-ms 未更新
- 反馈出现 NaN/Inf
- 电机报错
- 位置超出 q0 +/- (amp + pos-margin)
- 速度或力矩连续 3 帧超过阈值
- Ctrl+C

程序只向被测关节发送报文, 不会控制其它电机。正常完成退出码为 0, 中止为 3。

## 输出
输出目录为 `<out>/<日期_时间>_j<关节下标>/`, 包含:
- ticks.csv: 每拍一行
- meta.json: 实验参数、q0、零位偏移、反馈频率、是否中止及原因

ticks.csv 各列(时间均为相对第一拍的纳秒):

| 列 | 含义 |
|---|---|
| tick / phase | 拍序号 / 所处阶段 |
| t_tick_ns | 本拍计划起始时刻 |
| t_read_ns | 读取反馈的时刻 |
| t_cmd_ns | 指令交给底层库的时刻, 等同于部署时 Python 调 set_motor_cmd 的时刻 |
| q_des, qd_des, kp, kd, tau_ff | 本拍下发的指令 |
| q, qd, tau, temperature, error | 最近一帧电机回传 |
| rx_ns / rx_count | 该帧回传被解包的时刻 / 回传帧计数 |
| tx_ns / tx_count | 最近一次向该电机写 CAN 报文的时刻 / 发送计数(诊断用) |

时间戳对齐方式:
- 指令序列: 每一行取 (t_cmd_ns, q_des)
- 测量序列: 只取 rx_count 比上一行增加的行, 取 (rx_ns, q, qd, tau), 每帧回传恰好一条

```python
import pandas as pd
df = pd.read_csv("ticks.csv")
cmd = df[["t_cmd_ns", "q_des", "kp", "kd"]]
meas = df[df["rx_count"].diff() > 0][["rx_ns", "q", "qd", "tau"]]
```

## 注意事项
- 指令从 t_cmd_ns 到实际上总线约有 0-6 ms 延迟(1 kHz 打包线程 + 1 kHz 发送线程 + 每个电机每 4 ms 轮到一次), 这部分与部署链路一致, 交给 PACE 作为执行器延迟辨识。
- 电机为应答模式, 发一帧指令才回一帧状态。电机 ID 1-4、7-10、16-29 的回传约 250 Hz, 5/6/11/12 与 41-43 约 1 kHz。
- 角度坐标系与 get_motor_data 相同: 电机角 - lf1_zero_offset_rad[下标], 未乘 low_control.py 中的 motor_direction / leg_offset, 转 URDF 坐标需离线处理。
- 并联关节(踝 4/5/10/11、腰 13/14)的 PD 在 C 端做关节空间计算、解算与限幅, 本程序不支持。
- 使用 sudo 运行, 输出文件属主为 root。

#变更日志
- 2026-09-28: 新增 pace_single_joint(PACE 单关节 chirp 采集); 底层库增加逐关节收发时间戳(encos/pace_stamp.*)。

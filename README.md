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

## 数据检查
用 scripts/analyze_ticks.py 检查一次采集(需要 numpy、matplotlib; 同目录下必须有 meta.json):
```
python3 scripts/analyze_ticks.py build/pace_log/<运行目录>
```
输出:
- 指令/测量/发送帧数
- 发送与回传间隔分布, 以及回传间隔 > 6 ms 时丢帧发生在发送端还是回传端
- 发送 -> 回传解包延迟
- 控制循环抖动
- chirp 段闭环频率响应, 以及惯量 I、粘滞阻尼 b、库伦摩擦 Fc、延迟 T 的粗估计(仅用于检查数据, 正式辨识交给 PACE)

图片 analysis_time_<下标>.png / analysis_timing_<下标>.png / analysis_bode_<下标>.png 保存在运行目录(可用 --out 指定)。

加 --urdf 时, 模型计入 URDF 算出的重力刚度(吊装时髋 pitch/roll 约 19 Nm/rad, 与 kp 同量级), 并给出连杆惯量与 armature(= 拟合总惯量 - 连杆惯量)估计; armature 为负说明该次数据不可信或基座在晃动:
```
python3 scripts/analyze_ticks.py build/pace_log/<运行目录> --urdf urdf/keenon_l1.urdf
```
单独查看各关节吊装姿态下的连杆惯量、重力刚度和限位(--pose 可传 meta.json 使用实测姿态):
```
python3 scripts/urdf_dynamics.py urdf/keenon_l1.urdf
```pace_chirp 的数据默认只分析被激励关节, --joint 只看某一个, --all 同时看保持关节。

## 注意事项
- 指令从 t_cmd_ns 到实际上总线约有 0-6 ms 延迟(1 kHz 打包线程 + 1 kHz 发送线程 + 每个电机每 4 ms 轮到一次), 这部分与部署链路一致, 交给 PACE 作为执行器延迟辨识。
- 电机为应答模式, 发一帧指令才回一帧状态。电机 ID 1-4、7-10、16-29 的回传约 250 Hz, 5/6/11/12 与 41-43 约 1 kHz。
- 角度坐标系与 get_motor_data 相同: 电机角 - lf1_zero_offset_rad[下标], 未乘 low_control.py 中的 motor_direction / leg_offset, 转 URDF 坐标需离线处理。
- 并联关节(踝 4/5/10/11、腰 13/14)的 PD 在 C 端做关节空间计算、解算与限幅, 本程序不支持。
- 使用 sudo 运行, 输出文件属主为 root。

# PACE 多关节采集(pace_chirp)
正式采集用 pace_chirp, 生成的数据可直接转换为 pace-sim2real 的 chirp_data.pt。pace_single_joint 只用于验证时序。

与 pace-sim2real 的 data_collection.py 一致: 多个关节同时激励, 共用同一条线性 chirp 相位, 每个关节有自己的振幅(可带符号)和中心位置。吊装时各肢体之间动力学不耦合, 默认用 PD 保持被激励关节所在肢体的其余关节, 其它肢体不发送报文。支持整条腿(含并联踝)一起采集, 见下文「整条腿采集」。

## 与 pace_single_joint 的区别
- 输入输出都是 URDF(策略)坐标, 换算与部署链路一致: q_urdf = (q_api * motor_direction + pos_offset) * dance_dir, 不加 leg_offset(零偏交给 PACE 辨识)。motor_direction / dance_dir / pos_offset 抄自 deploy_real, 修改部署代码时要同步修改 src/pace_chirp.cpp。
- kp/kd 从增益文件读取, 与策略一致。
- 可以把被激励关节先移动到指定中心位置再做 chirp(例如让膝关节离开伸直限位)。
- 支持串联关节和并联踝(4/5/10/11), 踝与部署链路一样做关节空间 PD。并联腰(13/14)只支持 PD 保持(见「固定头部采集」), 不参与时不发送指令, 处于无力状态, PACE 仿真中要设为零刚度。

## 步骤
1. 增益文件: 采集与重新训练统一使用 gains_g1_legs.txt(腿部为 G1 取整增益: 髋 pitch/yaw 40/3、髋 roll/膝 100/6、踝 30/2; 腰与手臂同 gains_yaoguwu_plus_122500i.txt)。gains_yaoguwu_plus_122500i.txt 为当前已部署策略的增益(腿部 kd/kp = 0.2, 阻尼过重, 惯量与延迟难以辨识), 其采集数据可作交叉验证。从策略导出增益(有 onnxruntime / onnx 时优先使用, 没有也能直接解析):
```
python3 scripts/export_gains.py <policy.onnx> -o gains_<名称>.txt
```
2. dry-run 检查参数, 例如激励整条右腿的 4 个串联关节, 膝关节中心设在 0.4 rad:
```
sudo ./pace_chirp --gains ../gains_g1_legs.txt --joints 6,7,8,9 --amps 0.1,0.05,0.05,0.1 --centers q0,q0,q0,0.4 --f1 5 --duration 30 --dry-run
```
3. 吊装后真机运行: 同样的命令, 把 --dry-run 换成 --confirm-suspended。
4. 检查数据:
```
python3 scripts/analyze_ticks.py build/pace_log/<运行目录>
```
5. 转换为 PACE 格式(--dt 为 PACE 仿真的 physics dt, --joint-order 与 PACE 环境的 sim2real.joint_order 一致):
```
python3 scripts/to_pace.py build/pace_log/<运行目录> --dt 0.005 --joint-order right_hip_pitch_joint,right_hip_roll_joint,right_hip_yaw_joint,right_knee_joint
```
输出 chirp_data.pt({"time", "dof_pos", "des_dof_pos"}, URDF 坐标)和 chirp_data_info.json(关节顺序、增益、初始位置、需要在仿真中复现的保持/无力关节)。

## 主要参数
| 参数 | 说明 | 默认值 |
|---|---|---|
| --gains | 增益文件, 每行 `下标 关节名 kp kd` | 必填 |
| --joints | 被激励关节下标, 逗号分隔 | 必填 |
| --amp / --amps | 统一振幅 / 逐关节振幅(可为负) [rad], 上限 0.3 | 0.05 |
| --centers | 逐关节 chirp 中心 [rad, URDF], q0 表示当前位置 | 全部 q0 |
| --hold | PD 保持的关节: limb / none / 下标列表 | limb |
| --move-time | 在 q0 与中心位置之间移动的时长 [s] | 2.0 |
| --max-move | 中心位置距 q0 的上限 [rad] | 0.5 |
| --max-err | \|q - q_des\| 超过即中止 [rad], 至少 2 倍最大振幅 + 0.05 | 0.25 |
| --max-tau | 力矩安全阈值 [Nm], 踝为解算后的关节力矩 | 30 |
| --ankle-kd-ff | 踝电机侧附加阻尼 kd_ff[0](电机空间) | 0 |

其余参数(--f0/--f1/--duration/--ramp/--hold-pre/--hold-post/--rate/--max-vel/--stale-ms/--out)与 pace_single_joint 相同, 完整说明见 `./pace_chirp --help`。

运行阶段: probe -> engage -> move_in(移动到中心) -> hold_pre -> chirp -> hold_post -> move_out(回到 q0) -> release -> zero。中止条件与 pace_single_joint 相同, 另增加跟踪误差超限。

ticks.csv 中每个参与关节 i 有一组列: q_des_i, kp_i, kd_i, q_i, qd_i, tau_i, temperature_i, error_i, rx_ns_i, rx_count_i, tx_ns_i, tx_count_i。
有踝/腰参与时另有并联电机原始回传列 motor_q_m, motor_qd_m, motor_tau_m, motor_rx_ns_m, motor_rx_count_m(m 为 4/5、10/11 或 13/14, get_motor_data 同坐标, 未做并联解算), 用于离线重新解算。

## 逐关节采集脚本(躯干吊装、无法刚性固定)
训练与采集统一使用 G1 增益 gains_g1_legs.txt(脚本默认值)。髋 yaw、膝、踝每次只激励一个关节(同腿其余关节 PD 保持), 依次跑完两条腿; 髋 pitch / 髋 roll 腿部惯量大, 单腿运动会让吊装的躯干明显摆动(实测拟合出的总惯量远小于 URDF 连杆惯量), 因此两腿镜像同时激励, 每批只跑一次(髋 pitch 用 anti 抵消俯仰反力矩, 髋 roll 用 sym 抵消左右力)。每次运行前询问, 中止时可重试/跳过/退出:
```
scripts/pace_single_joints.sh --dry-run            # 先检查全部参数
sudo scripts/pace_single_joints.sh                 # 右腿 -> 左腿, f1 按关节表, duration=30 s
sudo scripts/pace_single_joints.sh --legs right --only hip_yaw,knee
```
- JOINT_TABLE 按 G1 增益与 URDF 模型设计(依据见脚本开头注释):

| 关节 | 振幅 | 中心 | f1 | 模式 | 预测自然频率 / 阻尼比 |
|---|---|---|---|---|---|
| 髋 pitch | 0.20 | q0 | 3 Hz | anti(两腿) | 1.24 Hz / 0.20; 实测谐振远弱于预测(振幅 0.2 时最大误差 0.26 rad) |
| 髋 roll | 0.05 | q0 | 4 Hz | sym(两腿) | 1.87 Hz / 0.30, 谐振处放大约 1.5 倍 |
| 髋 yaw | 0.05 | q0 | 8 Hz | 单腿 | 3.7 Hz / 0.86 |
| 膝 | 0.10 | 0.3 | 6 Hz | 单腿 | 3.2 Hz / 0.56 |
| 踝 pitch | 0.05 | q0 | 8 Hz | 单腿, --max-vel 3.5 | 3.5 Hz / 0.72 |
| 踝 roll | 0.04 | q0 | 10 Hz | 单腿, --max-vel 3.5 | 5.0 Hz / 1.03 |

- 输出在 build/pace_log/<日期_时间>_single/ 下, 每次运行一个目录(镜像运行的 tag 为 m_hip_pitch / m_hip_roll), 另有 <tag>.log 与 summary.txt(每次运行的振幅、f1、模式、退出码和目录)。
- 转换时 --joint-order 只写该次激励的关节, 例如 `python3 scripts/to_pace.py <目录> --dt 0.005 --joint-order right_knee_joint`。
- 左膝中心 0.3 假设两腿膝关节正方向均为弯曲, 首次运行左膝时观察 move_in 阶段, 方向不对立即 Ctrl+C。
- --amp X 覆盖关节表中的振幅(配合 --only); --max-err 默认按每次的振幅取 max(0.25, 2*振幅+0.05)。

## 固定头部采集(--hold body)
吊装时躯干随反作用力摆动(实测髋 pitch 拟合惯量远小于 URDF, armature 为负), 固定基座仿真无法复现。URDF 中头部属于 torso_link(无颈关节), 固定头部即固定 torso_link; 骨盆经腰 yaw/roll/pitch 与躯干相连。--hold body 把两腿与腰(0-14)中未激励的关节全部 PD 保持, 腰 roll/pitch(并联)与踝相同, 由底层发送线程做关节空间 PD(关节力矩限幅 60 Nm, 电机力矩限幅 50 Nm, 电机侧阻尼 kd_ff[1] 由 --waist-kd-ff 设置, 默认 0)。手臂挂在固定的躯干上, 与腿部动力学无关, 不保持。
```
scripts/pace_single_joints.sh --dry-run --hold body --only hip_pitch,hip_roll
sudo scripts/pace_single_joints.sh --hold body --only hip_pitch,hip_roll
sudo scripts/pace_single_joints.sh --hold body --gains gains_g1_legs_stiff_waist.txt --only hip_pitch,hip_roll
```
- 骨盆加两腿约 19.5 kg, 吊在腰下方(重力刚度约 80 Nm/rad, 对腰 pitch/roll 轴惯量约 4.5 kg·m²): 默认腰增益(25/5)下腰 pitch/roll 模态约 0.77 Hz、阻尼比约 0.11, 落在 chirp 频带内, 骨盆仍会动。gains_g1_legs_stiff_waist.txt 腿部与 gains_g1_legs.txt 相同, 只把腰调到 yaw 100/5、roll/pitch 200/20 以减小骨盆运动; 首次运行先用默认增益确认腰 PD 正常, 再换刚度高的文件。
- PACE 仿真中固定 torso_link(不是 pelvis), 腰与两腿按采集时的增益 PD 保持(to_pace.py 输出的 chirp_data_info.json 中 held_not_in_order 与 waist 字段)。腰关节编码器直接记录了骨盆相对躯干的运动(analyze_ticks.py --all 可查看)。
- 每次运行的 probe 与 release/zero 阶段腰与腿均为零刚度: 躯干直立固定时下半身悬垂在腰下方, 处于稳定平衡, 不会坠落, 但会轻微摆动(腰 yaw 无重力恢复)。头部夹具须能承受整机重量与动态载荷, 吊绳建议保留但放松作为备份。
- 髋 pitch 仍建议 anti(两腿俯仰动量抵消, 腰 pitch 几乎不受激励; 代价是激励腰 yaw)。

## 两腿镜像采集(吊装时抵消反作用力)
躯干吊装(非刚性固定)时, 单腿运动的反作用力会让躯干晃动, 仿真中固定基座无法复现。用 --mirror 只写一条腿的参数, 程序自动生成另一条腿:
```
sudo ./pace_chirp --gains ../gains_g1_legs.txt --joints 0,1,2,3,4 --amps 0.05,0.05,0.05,0.1,0.05 \
  --centers q0,q0,q0,0.2,q0 --mirror sym --max-err 0.3 --f1 5 --duration 30 --dry-run
```
| 模式 | roll/yaw 类(髋 roll、髋 yaw、踝 roll) | pitch 类(髋 pitch、膝、踝 pitch) | 躯干受力 |
|---|---|---|---|
| sym | 镜像(同时外展/内收) | 两腿同向 | 左右、偏航抵消; 俯仰反力矩与前后力叠加 |
| anti | 镜像 | 两腿反相(一前一后) | 左右、俯仰抵消; 产生绕竖直轴的扭矩 |

- 中心位置(--centers)总是按几何镜像生成: pitch 类同值, roll/yaw 类取反, q0 仍为各自当前位置。
- 对侧符号按 src/pace_chirp.cpp 中 kMirrorGeomSign 计算, 假设左右同名关节 URDF 轴方向相同(G1 类约定)。首次使用先小振幅慢速运行(如 `--amp 0.03 --f1 0.5`)确认实际运动确为镜像, 不对时修改 kMirrorGeomSign。
- sym 与 anti 哪个躯干晃得少取决于吊装方式, 可在躯干上放手机记录 IMU 对比。
- 只能写一条腿的关节(0-5 或 6-11), 输出目录与 meta.json 中记录 mirror 模式。

## 整条腿采集
一次激励一条腿的全部 6 个关节(髋 pitch/roll/yaw、膝、踝 pitch/roll), 例如右腿, 膝中心设在 0.4 rad:
```
sudo ./pace_chirp --gains ../gains_g1_legs.txt --joints 6,7,8,9,10,11 --amps 0.05,0.05,0.05,0.1,0.05,0.05 --centers q0,q0,q0,0.4,q0,q0 --f1 5 --duration 30 --dry-run
```
左腿为 `--joints 0,1,2,3,4,5`。确认无误后把 --dry-run 换成 --confirm-suspended。转换时关节顺序同样与 PACE 环境一致:
```
python3 scripts/to_pace.py build/pace_log/<运行目录> --dt 0.005 --joint-order right_hip_pitch_joint,right_hip_roll_joint,right_hip_yaw_joint,right_knee_joint,right_ankle_pitch_joint,right_ankle_roll_joint
```

踝(并联)的处理:
- 指令: 与部署链路完全相同。下发关节空间 q_des/kp/kd, 底层 1 kHz 发送线程用并联解算后的关节状态算 PD 力矩(关节力矩限幅 pitch 60 / roll 20 Nm), 经雅可比换成两个电机的力矩(电机力矩限幅 25 Nm), 电机侧阻尼为 kd_ff[0](--ankle-kd-ff, 默认 0, 即只有关节 PD, 与仿真中的 PD 一致; 要与部署一致时填部署所用的值)。
- 反馈: 程序用两个踝电机各自带时间戳的回传做正解, 得到 pitch/roll 的 q/qd/tau, 解算与 get_motor_data 相同。rx_ns 取两个电机中较新的一帧, rx_count 取较小者(两个电机都有新帧才算一帧新测量), 超时检查按较旧的一帧。
- 同一只脚的 pitch/roll 共用两个电机, 必须一起参与: 只激励其中一个时, 另一个自动 PD 保持(--hold none 时也一样)。
- --hold limb(默认)现在也会保持同肢体的踝, 例如只激励右膝时右踝 PD 保持, 不再无力。
- 首次运行建议先只激励踝(如 `--joints 10,11 --amp 0.03 --f1 2`), 确认解算后的角度方向、跟踪正常, 再做整条腿。
- 转换得到的 chirp_data_info.json 中 ankle 字段记录踝关节、kd_ff 与力矩限幅, 仿真中如需复现可参考。
- 同一条 chirp 同时激励踝 pitch 和 roll 且振幅相同时, 运动全部落在一个电机上(pitch 对应两电机同向转, roll 对应反向转), 另一个电机几乎不动、得不到激励。整条腿采集时只激励踝 pitch(roll 自动保持), 再单独采一次 roll(`--joints 11`)。
- 底层 fk 的收敛阈值较松(杆长误差约 0.1 mm), 以上一帧解为初值时关节角呈台阶状, 误差约 1 mrad RMS、最大约 3 mrad(部署链路同样如此)。to_pace.py 默认用 motor_q_* 列按严格收敛重新正解踝角度(scripts/ankle_fk.py), --ankle-fk lib 则使用采集时记录的值; 两者差异打印在终端并写入 chirp_data_info.json 的 ankle.dof_pos_fk。

#变更日志
- 2026-09-28: 新增 pace_single_joint(PACE 单关节 chirp 采集); 底层库增加逐关节收发时间戳(encos/pace_stamp.*)。
- 2026-09-29: 新增 pace_chirp(多关节 PACE 采集, URDF 坐标)、scripts/export_gains.py、scripts/to_pace.py; analyze_ticks.py 支持 pace_chirp 输出。
- 2026-09-29: 修复发送线程偶尔连续多轮漏发同一组电机的问题(实测单个电机最长约 50ms 收不到指令): 发送队列为空时重发不超过 3ms 的上一份指令快照(encos/transmit_fd.cpp)。此修改同样作用于部署用的 libkeenon_lf1.so。
- 2026-09-29: pace_chirp 支持并联踝(4/5/10/11), 可整条腿一起采集; --hold limb 同时保持同肢体的踝; 新增 --ankle-kd-ff; ParallelMechanism 增加不依赖全局状态的 motorToJointAnkle(motorToJointLeft/Right 改为调用它, 结果不变)。
- 2026-09-29: to_pace.py 默认用踝电机原始角严格正解踝 pitch/roll(scripts/ankle_fk.py), 消除底层 fk 收敛阈值造成的约 1-3 mrad 台阶误差; 底层解算未改动。
- 2026-09-30: pace_chirp 新增 --mirror sym/anti, 只写一条腿的参数即自动生成另一条腿的镜像/反相运动, 用于吊装时抵消反作用力。
- 2026-09-30: 新增 scripts/pace_single_joints.sh, 依次对两条腿逐关节 chirp 采集。
- 2026-10-08: 训练与采集改用 G1 增益(gains_g1_legs.txt); 新增 urdf/keenon_l1.urdf 与 scripts/urdf_dynamics.py(吊装姿态下的连杆惯量、重力刚度); analyze_ticks.py 新增 --urdf(模型计入重力刚度, 给出 armature 估计); pace_single_joints.sh 按 URDF 模型逐关节设定振幅与 f1, 髋 pitch/roll 改为两腿镜像(anti/sym)。
- 2026-10-09: pace_chirp 支持 PD 保持并联腰(13/14, 与踝相同的关节空间 PD), 新增 --hold body(两腿与腰全部保持, 用于固定头部采集)与 --waist-kd-ff; ParallelMechanism 增加不依赖全局状态的 motorToJointWaist(motorToJointW 改为调用它, 结果不变); pace_single_joints.sh 新增 --hold / --amp / --max-err / --waist-kd-ff, max-err 按振幅自动设定, 髋 pitch 振幅改为 0.20; 新增 gains_g1_legs_stiff_waist.txt。

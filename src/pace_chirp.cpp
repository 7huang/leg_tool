/*
 * pace_chirp: PACE 真机采集 —— 多关节同时 chirp 激励, 同时记录全部参与关节的下发指令与电机回传。
 *
 * 激励方式与 pace-sim2real 的 scripts/pace/data_collection.py 一致: 所有被激励关节共用同一条线性 chirp 相位,
 * 每个关节有自己的振幅(可带符号)和中心位置。
 * 吊装(基座固定)时各肢体之间动力学不耦合, 默认只用 PD 保持"被激励关节所在肢体"的其余关节,
 * 其它肢体不发送任何报文。
 *
 * 踝(并联, 4/5/10/11): 与部署链路相同, 下发关节空间 q_des/kp/kd, 由底层发送线程用并联解算后的关节状态
 *   计算关节 PD 力矩, 再经雅可比换成两个电机的力矩(关节力矩限幅 pitch 60 / roll 20 Nm, 电机力矩限幅 25 Nm),
 *   电机侧另加阻尼 kd_ff[0](--ankle-kd-ff)。同一只脚的 pitch/roll 共用两个电机, 必须同时参与。
 *   记录用的关节反馈由本程序用两个电机各自带时间戳的回传做正解得到, 与底层 get_motor_data 的解算相同。
 * 腰 roll/pitch(并联, 13/14): 只支持 PD 保持(不支持激励), 处理方式与踝相同: 底层发送线程做关节空间 PD
 *   (关节力矩限幅 60 Nm, 电机力矩限幅 50 Nm), 电机侧另加阻尼 kd_ff[1](--waist-kd-ff), 两者必须同时参与。
 *   用于固定头部(即 torso_link)时 PD 保持腰, 让骨盆尽量不动; PACE 仿真中固定 torso_link, 腰按相同 PD 建模。
 *   不参与时不发送指令, 处于无力状态, PACE 仿真中需要把它们设成零刚度。
 *
 * 坐标: 命令行输入与输出均为 URDF(策略)坐标, 换算与部署链路一致:
 *   low_control.py      : q_low  = (q_api - leg_offset) * motor_direction
 *   deploy_real/task 下 : q_urdf = (q_low + pos_offset) * dance_dir
 *   本程序 leg_offset = 0, 零偏交给 PACE 辨识。
 *
 * 阶段: probe -> engage -> move_in -> hold_pre -> chirp -> hold_post -> move_out -> release -> zero -> done
 *   probe   : 零刚度 + 阻尼, 读取各关节当前位置 q0(要求静止)
 *   engage  : kp 平滑升到目标, 目标位置保持 q0
 *   move_in : 被激励关节从 q0 平滑移动到中心位置 center
 *   chirp   : q_des = center + amp * 包络 * sin(相位)
 *   move_out: 被激励关节回到 q0, 然后 release 平滑卸掉 kp
 *   任何安全检查失败或 Ctrl+C: damp(kp=0, 只留阻尼, 不注入能量) -> zero -> done
 *
 * 时间戳定义与 pace_single_joint 相同(CLOCK_MONOTONIC, 相对第一拍的纳秒):
 *   t_cmd_ns 指令交给底层库的时刻; rx_ns_<i> 回传帧解包时刻; tx_ns_<i> 最近一次写 CAN 的时刻
 *
 * 输出: <out>/<日期_时间>_<tag>/ticks.csv 与 meta.json, 转换为 PACE 格式见 scripts/to_pace.py
 */
#include "ec_api_without_ros.hpp"
#include "pace_stamp.h"
#include "parallelmechanism.h"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <string>
#include <tuple>
#include <vector>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace {

constexpr int kN = 29;

// 与 deploy_real/task/*.py 的 joint_xml 一致(即 MotorCmd 数组下标顺序)
const char *kJointNames[kN] = {
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint", "left_knee_joint",
    "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint", "right_knee_joint",
    "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint",
};

// 以下三组数组必须与部署代码保持一致, 修改部署代码时同步修改这里
// deploy_real/low_control.py: motor_direction
const int kMotorDirection[kN] = {1, -1, 1, -1, 1, 1,
                                 1, -1, 1, -1, 1, 1,
                                 1, -1, -1,
                                 1, 1, 1, 1, -1, 1, 1,
                                 1, 1, 1, 1, -1, 1, 1};
// deploy_real/task/*.py: dance_dir
const int kDanceDir[kN] = {1, -1, -1, 1, 1, 1,
                           -1, -1, -1, -1, 1, 1,
                           1, 1, 1,
                           1, -1, -1, 1, -1, -1, -1,
                           -1, -1, -1, -1, -1, 1, -1};
// deploy_real/task/*.py: pos_offset
const double kPosOffset[kN] = {0, 0, 0, 0, 0, 0,
                               0, 0, 0, 0, 0, 0,
                               0, 0, 0,
                               0, -1.57, 0, 1.57, 0, 0, 0,
                               0, 1.57, 0, -1.57, 0, 0, 0};

const char *kLimbName[] = {"left_leg", "right_leg", "waist", "left_arm", "right_arm"};

// --mirror: 腿内顺序 hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll。
// 左右腿几何镜像时对侧关节角(URDF)的符号, 假设左右同名关节轴方向相同(G1 类 URDF 的约定):
// pitch 类(髋/膝/踝 pitch)同号, roll/yaw 类反号。换机器人或 URDF 时须重新确认。
const int kMirrorGeomSign[6] = {1, -1, -1, 1, 1, -1};
// pitch 类关节: anti 模式下对侧反相
const bool kMirrorPitchLike[6] = {true, false, false, true, true, false};

enum Phase : int { PROBE = 0, ENGAGE, MOVE_IN, HOLD_PRE, CHIRP, HOLD_POST, MOVE_OUT, RELEASE, DAMP, ZERO, DONE };
const char *kPhaseName[] = {"probe", "engage", "move_in", "hold_pre", "chirp", "hold_post",
                            "move_out", "release", "damp", "zero", "done"};

constexpr double kProbeMinS = 0.5;         // 探测阶段最短时长 [s]
constexpr double kProbeMaxS = 3.0;         // 探测阶段超时 [s]
constexpr double kProbeSettleS = 0.2;      // 探测开始后丢弃的反馈时长 [s]
constexpr uint32_t kProbeMinSamples = 50;  // 每个关节计算 q0 至少需要的新反馈帧数
constexpr double kProbeMaxSpread = 0.02;   // 探测期关节应静止, q 极差上限 [rad]
constexpr double kEngageS = 0.5;
constexpr double kReleaseS = 0.5;
constexpr double kDampS = 0.5;
constexpr double kZeroS = 0.1;
constexpr int kViolationFrames = 3;        // 速度/力矩连续超限的反馈帧数才中止, 过滤单帧毛刺

double api_to_urdf(int i, double q_api){
    return (q_api * kMotorDirection[i] + kPosOffset[i]) * kDanceDir[i];
}

double urdf_to_api(int i, double q_urdf){
    return (q_urdf * kDanceDir[i] - kPosOffset[i]) * kMotorDirection[i];
}

// 速度、力矩只有方向换算
double sign_api_to_urdf(int i){
    return kMotorDirection[i] * kDanceDir[i];
}

// 共用两个电机的并联关节组: 0 左踝(4 pitch, 5 roll), 1 右踝(10 pitch, 11 roll), 2 腰(13 roll, 14 pitch)。
// 组内两个关节下标同时也是两个电机的下标; 返回 -1 表示不是并联关节
constexpr int kParGroups = 3;
constexpr int kWaistGroup = 2;
const int kParIndex[kParGroups][2] = {{4, 5}, {10, 11}, {13, 14}};

bool is_parallel_joint(int i){
    return i == 4 || i == 5 || i == 10 || i == 11 || i == 13 || i == 14;
}

int par_group(int i){
    for (int g = 0; g < kParGroups; ++g){
        if (i == kParIndex[g][0] || i == kParIndex[g][1]) return g;
    }
    return -1;
}

int par_partner(int i){
    const int g = par_group(i);
    return i == kParIndex[g][0] ? kParIndex[g][1] : kParIndex[g][0];
}

// 本程序不支持激励的关节: 并联腰(只能 PD 保持)
bool is_waist_parallel(int i){
    return i == 13 || i == 14;
}

int limb_of(int i){
    if (i < 6) return 0;
    if (i < 12) return 1;
    if (i < 15) return 2;
    if (i < 22) return 3;
    return 4;
}

struct Options {
    std::string gains_path;
    std::vector<int> joints;
    std::vector<double> amps;     // 为空时全部使用 amp
    std::vector<double> centers;  // NAN 表示使用 q0
    std::string hold = "limb";
    std::string mirror = "none";  // none / sym / anti
    std::string tag;
    double amp = 0.05;
    double f0 = 0.1;
    double f1 = 2.0;
    double duration = 20.0;
    double ramp = 1.0;
    double hold_pre = 1.0;
    double hold_post = 1.0;
    double move_time = 2.0;
    double max_move = 0.5;
    int rate = 1000;
    double max_vel = 5.0;
    double max_tau = 30.0;
    double max_err = 0.25;
    double stale_ms = 50.0;
    double ankle_kd_ff = 0.0;
    double waist_kd_ff = 0.0;
    std::string out = "./pace_log";
    bool confirm = false;
    bool dry_run = false;
};

struct Joint {
    int index = -1;
    bool excited = false;
    double kp = 0.0;
    double kd = 0.0;
    double amp = 0.0;
    double center = NAN;  // URDF 坐标, NAN 表示使用 q0
    double q0 = 0.0;
    double probe_sum = 0.0;
    double probe_min = 1e9;
    double probe_max = -1e9;
    uint32_t probe_n = 0;
    uint32_t last_rx_count = 0;
    int vel_bad = 0;
    int tau_bad = 0;
    double last_q_des = 0.0;
};

struct TickRec {
    int64_t t_tick_ns;
    int64_t t_read_ns;
    int64_t t_cmd_ns;
    uint32_t tick;
    int32_t phase;
};

// 除 rx_ns/tx_ns/计数外均为 URDF 坐标
struct JointRec {
    float q_des;
    float kp;
    float kd;
    float q;
    float qd;
    float tau;
    int64_t rx_ns;
    int64_t tx_ns;
    uint32_t rx_count;
    uint32_t tx_count;
    uint8_t temperature;
    uint8_t error;
};

// 并联(踝/腰)电机原始回传(get_motor_data 同坐标: 电机角 - 零偏, 未做并联解算), 供离线重新解算
struct MotorRec {
    float q;
    float qd;
    float tau;
    int64_t rx_ns;
    uint32_t rx_count;
};

std::atomic<bool> g_stop{false};

void on_signal(int){
    g_stop = true;
}

void print_usage(const char *prog){
    printf("用法: sudo %s --gains <gains.txt> --joints <i,j,...> [选项] --confirm-suspended\n"
           "  --gains FILE       各关节 kp/kd, 由 scripts/export_gains.py 从策略 ONNX 导出\n"
           "  --joints LIST      被激励的关节下标(逗号分隔), 支持串联关节与踝(4/5/10/11), 腰 13/14 只能保持\n"
           "  --amp X            所有被激励关节的 chirp 振幅 [rad], 默认 0.05\n"
           "  --amps LIST        逐关节振幅(可为负, 决定起始方向), 覆盖 --amp\n"
           "  --centers LIST     逐关节 chirp 中心 [rad, URDF 坐标], q0 表示当前位置, 默认全部 q0\n"
           "  --hold SPEC        PD 保持的关节: limb(默认, 同肢体其余关节) / body(两腿与腰 0-14 中未激励的关节,\n"
           "                     固定头部时使用) / none / 下标列表\n"
           "                     踝、腰的 pitch/roll 共用电机, 只列其中一个时另一个自动 PD 保持\n"
           "  --f0 X --f1 X      起止频率 [Hz], 默认 0.1 -> 2.0, 上限 10\n"
           "  --duration X       chirp 时长 [s], 默认 20\n"
           "  --ramp X           chirp 首尾余弦淡入淡出 [s], 默认 1.0\n"
           "  --hold-pre X       chirp 前保持 [s], 默认 1.0\n"
           "  --hold-post X      chirp 后保持 [s], 默认 1.0\n"
           "  --move-time X      q0 与中心位置之间移动的时长 [s], 默认 2.0\n"
           "  --max-move X       中心位置与 q0 的最大距离 [rad], 默认 0.5\n"
           "  --rate N           控制/记录频率 [Hz], 默认 1000, 范围 100-1000\n"
           "  --max-vel X        速度安全阈值 [rad/s], 默认 5\n"
           "  --max-tau X        力矩安全阈值 [Nm], 默认 30\n"
           "  --max-err X        |q - q_des| 超过该值即中止 [rad], 默认 0.25\n"
           "  --stale-ms X       反馈超过该时长未更新即中止 [ms], 默认 50\n"
           "  --ankle-kd-ff X    踝电机侧附加阻尼 kd_ff[0](电机空间), 默认 0; 与部署一致时填部署值\n"
           "  --waist-kd-ff X    腰 roll/pitch 电机侧附加阻尼 kd_ff[1](电机空间), 默认 0\n"
           "  --out DIR          输出根目录, 默认 ./pace_log\n"
           "  --tag S            输出目录名后缀, 默认 j<下标列表>\n"
           "  --mirror MODE      只写一条腿的 --joints/--amps/--centers, 自动生成另一条腿(吊装时抵消反作用力):\n"
           "                     sym  几何镜像: roll/yaw 类对称, pitch 类两腿同向(俯仰反力矩叠加)\n"
           "                     anti roll/yaw 类对称, pitch 类两腿反相(俯仰反力矩抵消, 但产生绕竖直轴的扭矩)\n"
           "                     中心位置总是几何镜像; 默认 none\n"
           "  --dry-run          不连接电机, 只检查参数并生成 plan.csv\n"
           "  --confirm-suspended 确认已吊装(或固定头部)、急停可用(真机运行必需)\n", prog);
}

std::vector<std::string> split(const std::string &s){
    std::vector<std::string> out;
    size_t start = 0;
    while (start <= s.size()){
        size_t end = s.find(',', start);
        if (end == std::string::npos) end = s.size();
        out.push_back(s.substr(start, end - start));
        start = end + 1;
    }
    return out;
}

bool parse_int_list(const std::string &s, std::vector<int> &out){
    for (const std::string &item : split(s)){
        char *end = nullptr;
        const long v = strtol(item.c_str(), &end, 10);
        if (item.empty() || *end != '\0') return false;
        out.push_back((int)v);
    }
    return true;
}

// allow_q0: 允许写 q0, 表示使用当前位置(记为 NAN)
bool parse_double_list(const std::string &s, std::vector<double> &out, bool allow_q0){
    for (const std::string &item : split(s)){
        if (allow_q0 && item == "q0"){
            out.push_back(NAN);
            continue;
        }
        char *end = nullptr;
        const double v = strtod(item.c_str(), &end);
        if (item.empty() || *end != '\0') return false;
        out.push_back(v);
    }
    return true;
}

bool parse_args(int argc, char **argv, Options &o){
    for (int i = 1; i < argc; ++i){
        const std::string key = argv[i];
        if (key == "-h" || key == "--help") return false;
        if (key == "--confirm-suspended"){ o.confirm = true; continue; }
        if (key == "--dry-run"){ o.dry_run = true; continue; }
        if (i + 1 >= argc){
            fprintf(stderr, "参数 %s 缺少取值\n", key.c_str());
            return false;
        }
        const char *v = argv[++i];
        bool ok = true;
        if      (key == "--gains")      o.gains_path = v;
        else if (key == "--joints")     ok = parse_int_list(v, o.joints);
        else if (key == "--amps")       ok = parse_double_list(v, o.amps, false);
        else if (key == "--centers")    ok = parse_double_list(v, o.centers, true);
        else if (key == "--hold")       o.hold = v;
        else if (key == "--tag")        o.tag = v;
        else if (key == "--mirror")     o.mirror = v;
        else if (key == "--amp")        o.amp = strtod(v, nullptr);
        else if (key == "--f0")         o.f0 = strtod(v, nullptr);
        else if (key == "--f1")         o.f1 = strtod(v, nullptr);
        else if (key == "--duration")   o.duration = strtod(v, nullptr);
        else if (key == "--ramp")       o.ramp = strtod(v, nullptr);
        else if (key == "--hold-pre")   o.hold_pre = strtod(v, nullptr);
        else if (key == "--hold-post")  o.hold_post = strtod(v, nullptr);
        else if (key == "--move-time")  o.move_time = strtod(v, nullptr);
        else if (key == "--max-move")   o.max_move = strtod(v, nullptr);
        else if (key == "--rate")       o.rate = atoi(v);
        else if (key == "--max-vel")    o.max_vel = strtod(v, nullptr);
        else if (key == "--max-tau")    o.max_tau = strtod(v, nullptr);
        else if (key == "--max-err")    o.max_err = strtod(v, nullptr);
        else if (key == "--stale-ms")   o.stale_ms = strtod(v, nullptr);
        else if (key == "--ankle-kd-ff") o.ankle_kd_ff = strtod(v, nullptr);
        else if (key == "--waist-kd-ff") o.waist_kd_ff = strtod(v, nullptr);
        else if (key == "--out")        o.out = v;
        else {
            fprintf(stderr, "未知参数: %s\n", key.c_str());
            return false;
        }
        if (!ok){
            fprintf(stderr, "参数 %s 的列表格式错误: %s\n", key.c_str(), v);
            return false;
        }
    }
    return !o.gains_path.empty() && !o.joints.empty();
}

// --mirror: 按 kMirrorGeomSign 为 --joints 中的每个腿关节生成对侧关节, 追加到列表末尾
bool expand_mirror(Options &o){
    if (o.mirror == "none") return true;
    if (o.mirror != "sym" && o.mirror != "anti"){
        fprintf(stderr, "参数错误: --mirror 只能是 none / sym / anti\n");
        return false;
    }
    if (!o.amps.empty() && o.amps.size() != o.joints.size()){
        fprintf(stderr, "参数错误: --amps 的个数必须与 --joints 相同\n");
        return false;
    }
    if (!o.centers.empty() && o.centers.size() != o.joints.size()){
        fprintf(stderr, "参数错误: --centers 的个数必须与 --joints 相同\n");
        return false;
    }
    if (o.amps.empty()) o.amps.assign(o.joints.size(), o.amp);
    if (o.centers.empty()) o.centers.assign(o.joints.size(), NAN);
    const size_t n = o.joints.size();
    for (size_t k = 0; k < n; ++k){
        const int i = o.joints[k];
        if (i < 0 || i >= 12){
            fprintf(stderr, "参数错误: --mirror 只支持腿关节(0-11), %d 不是\n", i);
            return false;
        }
        const int j = i < 6 ? i + 6 : i - 6;
        if (std::find(o.joints.begin(), o.joints.end(), j) != o.joints.end()){
            fprintf(stderr, "参数错误: 使用 --mirror 时只写一条腿, %s 与 %s 同时出现\n", kJointNames[i], kJointNames[j]);
            return false;
        }
        const int pos = i % 6;
        const int geom = kMirrorGeomSign[pos];
        const int motion = (o.mirror == "anti" && kMirrorPitchLike[pos]) ? -geom : geom;
        o.joints.push_back(j);
        o.amps.push_back(o.amps[k] * motion);
        o.centers.push_back(std::isnan(o.centers[k]) ? NAN : o.centers[k] * geom);
        printf("[PACE] mirror(%s): %-26s amp %+.4f -> %-26s amp %+.4f\n", o.mirror.c_str(), kJointNames[i], o.amps[k],
               kJointNames[j], o.amps.back());
    }
    return true;
}

// 文件格式(每行): index name kp kd, # 开头为注释
bool load_gains(const std::string &path, double kp[kN], double kd[kN]){
    for (int i = 0; i < kN; ++i){
        kp[i] = NAN;
        kd[i] = NAN;
    }
    FILE *f = fopen(path.c_str(), "r");
    if (f == nullptr){
        fprintf(stderr, "无法打开增益文件 %s\n", path.c_str());
        return false;
    }
    char line[512];
    int lineno = 0;
    while (fgets(line, sizeof(line), f) != nullptr){
        ++lineno;
        const char *p = line;
        while (*p == ' ' || *p == '\t') ++p;
        if (*p == '#' || *p == '\n' || *p == '\r' || *p == '\0') continue;
        int idx = -1;
        char name[128];
        double a = 0.0, b = 0.0;
        if (sscanf(p, "%d %127s %lf %lf", &idx, name, &a, &b) != 4){
            fprintf(stderr, "%s:%d 格式错误, 应为: index name kp kd\n", path.c_str(), lineno);
            fclose(f);
            return false;
        }
        if (idx < 0 || idx >= kN || strcmp(name, kJointNames[idx]) != 0){
            fprintf(stderr, "%s:%d 下标 %d 与关节名 %s 不匹配\n", path.c_str(), lineno, idx, name);
            fclose(f);
            return false;
        }
        kp[idx] = a;
        kd[idx] = b;
    }
    fclose(f);
    return true;
}

// 根据参数构建参与关节列表(按下标排序), 失败时打印原因
bool build_joints(const Options &o, std::vector<Joint> &joints){
    auto fail = [](const std::string &msg){
        fprintf(stderr, "参数错误: %s\n", msg.c_str());
        return false;
    };
    if (!(o.f0 > 0 && o.f0 < o.f1 && o.f1 <= 10)) return fail("频率必须满足 0 < f0 < f1 <= 10 Hz");
    if (!(o.duration >= 2 && o.duration <= 120)) return fail("--duration 必须在 [2, 120] s");
    if (!(o.ramp >= 0.1 && 2 * o.ramp <= o.duration)) return fail("--ramp 必须 >= 0.1 s 且不超过 duration/2");
    if (!(o.hold_pre >= 0 && o.hold_pre <= 10 && o.hold_post >= 0 && o.hold_post <= 10)) return fail("--hold-pre/--hold-post 必须在 [0, 10] s");
    if (!(o.move_time >= 0.5 && o.move_time <= 10)) return fail("--move-time 必须在 [0.5, 10] s");
    if (!(o.max_move > 0 && o.max_move <= 1.0)) return fail("--max-move 必须在 (0, 1.0] rad");
    if (o.rate < 100 || o.rate > 1000) return fail("--rate 必须在 [100, 1000] Hz");
    if (!(o.max_vel > 0 && o.max_tau > 0 && o.max_err > 0 && o.stale_ms >= 10)) return fail("安全阈值必须为正, --stale-ms >= 10");
    if (!(o.ankle_kd_ff >= 0 && o.ankle_kd_ff <= 5)) return fail("--ankle-kd-ff 必须在 [0, 5]");
    if (!(o.waist_kd_ff >= 0 && o.waist_kd_ff <= 10)) return fail("--waist-kd-ff 必须在 [0, 10]");
    if (!o.amps.empty() && o.amps.size() != o.joints.size()) return fail("--amps 的个数必须与 --joints 相同");
    if (!o.centers.empty() && o.centers.size() != o.joints.size()) return fail("--centers 的个数必须与 --joints 相同");

    double kp[kN], kd[kN];
    if (!load_gains(o.gains_path, kp, kd)) return false;

    bool involved[kN] = {false};
    bool excited[kN] = {false};
    for (size_t k = 0; k < o.joints.size(); ++k){
        const int i = o.joints[k];
        if (i < 0 || i >= kN) return fail("--joints 下标必须在 [0, 28]");
        if (is_waist_parallel(i)) return fail(std::string("并联腰关节不支持激励(只能保持): ") + kJointNames[i]);
        if (excited[i]) return fail(std::string("--joints 中重复: ") + kJointNames[i]);
        excited[i] = true;
        involved[i] = true;
    }

    if (o.hold == "limb"){
        for (int i = 0; i < kN; ++i){
            if (excited[i]) continue;
            for (int j : o.joints){
                if (limb_of(i) == limb_of(j)) involved[i] = true;
            }
        }
    }
    else if (o.hold == "body"){
        // 两腿与腰: 固定头部(torso_link)时, 骨盆经腰关节与躯干相连, 两腿与腰全部 PD 保持; 手臂挂在固定的躯干上, 与腿无关
        for (int i = 0; i < 15; ++i){
            if (!excited[i]) involved[i] = true;
        }
    }
    else if (o.hold != "none"){
        std::vector<int> hold;
        if (!parse_int_list(o.hold, hold)) return fail("--hold 必须是 limb、body、none 或下标列表");
        for (int i : hold){
            if (i < 0 || i >= kN) return fail("--hold 下标必须在 [0, 28]");
            if (excited[i]) return fail(std::string("关节同时出现在 --joints 与 --hold: ") + kJointNames[i]);
            involved[i] = true;
        }
    }
    // 同一只脚(或腰)的 pitch/roll 由同两个电机驱动, 必须一起下发关节 PD
    for (int i = 0; i < kN; ++i){
        if (involved[i] && par_group(i) >= 0 && !involved[par_partner(i)]){
            involved[par_partner(i)] = true;
            printf("[PACE] 注意: %s 与 %s 共用电机, 后者自动加入 PD 保持\n", kJointNames[i], kJointNames[par_partner(i)]);
        }
    }

    double max_amp = 0.0;
    for (int i = 0; i < kN; ++i){
        if (!involved[i]) continue;
        Joint J;
        J.index = i;
        J.excited = excited[i];
        J.kp = kp[i];
        J.kd = kd[i];
        if (!(J.kp > 0 && J.kp <= 500 && J.kd >= 0 && J.kd <= 50)){
            return fail(std::string("增益文件中缺少或超出范围(kp (0,500], kd [0,50]): ") + kJointNames[i]);
        }
        if (J.excited){
            const size_t k = std::find(o.joints.begin(), o.joints.end(), i) - o.joints.begin();
            J.amp = o.amps.empty() ? o.amp : o.amps[k];
            J.center = o.centers.empty() ? NAN : o.centers[k];
            if (!(std::fabs(J.amp) > 0 && std::fabs(J.amp) <= 0.3)){
                return fail(std::string("振幅必须满足 0 < |amp| <= 0.3 rad: ") + kJointNames[i]);
            }
            max_amp = std::max(max_amp, std::fabs(J.amp));
        }
        joints.push_back(J);
    }
    if (max_amp * 2 * M_PI * o.f1 > 0.8 * o.max_vel) return fail("参考峰值速度 max|amp|*2*pi*f1 超过 0.8*max-vel, 请减小振幅或 f1");
    if (o.max_err < 2 * max_amp + 0.05) return fail("--max-err 必须 >= 2*max|amp| + 0.05(高频时跟踪误差可接近 2 倍振幅)");
    return true;
}

// 线性扫频单位信号, 首尾 ramp 秒余弦包络淡入淡出, 保证目标位置和速度在起止处连续
double chirp_unit(const Options &o, double t){
    if (t <= 0 || t >= o.duration) return 0.0;
    double env = 1.0;
    if (t < o.ramp){
        env = 0.5 * (1 - cos(M_PI * t / o.ramp));
    }
    else if (t > o.duration - o.ramp){
        env = 0.5 * (1 - cos(M_PI * (o.duration - t) / o.ramp));
    }
    const double phase = 2 * M_PI * (o.f0 * t + 0.5 * (o.f1 - o.f0) / o.duration * t * t);
    return env * sin(phase);
}

double smooth01(double x){
    if (x <= 0) return 0.0;
    if (x >= 1) return 1.0;
    return 0.5 * (1 - cos(M_PI * x));
}

void sleep_until_ns(int64_t t_ns){
    struct timespec ts;
    ts.tv_sec = t_ns / 1000000000LL;
    ts.tv_nsec = t_ns % 1000000000LL;
    while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr) == EINTR){
    }
}

bool mkdir_p(const std::string &path){
    std::string cur;
    for (size_t i = 0; i <= path.size(); ++i){
        if ((i == path.size() || path[i] == '/') && !cur.empty()){
            if (mkdir(cur.c_str(), 0755) != 0 && errno != EEXIST) return false;
        }
        if (i < path.size()) cur += path[i];
    }
    return true;
}

std::string make_run_dir(const Options &o){
    char stamp[32];
    time_t now = time(nullptr);
    strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", localtime(&now));
    std::string tag = o.tag;
    if (tag.empty()){
        tag = "j";
        for (size_t k = 0; k < o.joints.size(); ++k){
            tag += (k ? "-" : "") + std::to_string(o.joints[k]);
        }
    }
    const std::string dir = o.out + "/" + stamp + "_" + tag + (o.dry_run ? "_dry" : "");
    if (!mkdir_p(dir)){
        fprintf(stderr, "创建输出目录失败: %s (%s)\n", dir.c_str(), strerror(errno));
        return "";
    }
    return dir;
}

std::string json_escape(const std::string &s){
    std::string out;
    for (char c : s){
        if (c == '"' || c == '\\') out += '\\';
        out += c;
    }
    return out;
}

// 参与关节所在肢体中未参与的并联关节: 本程序不给它们发指令, PACE 仿真中需设成零刚度
std::vector<int> limp_parallel_joints(const std::vector<Joint> &joints){
    bool limb_used[5] = {false};
    bool used[kN] = {false};
    for (const Joint &J : joints){
        limb_used[limb_of(J.index)] = true;
        used[J.index] = true;
    }
    std::vector<int> out;
    for (int i = 0; i < kN; ++i){
        if (is_parallel_joint(i) && limb_used[limb_of(i)] && !used[i]) out.push_back(i);
    }
    return out;
}

bool has_group(const std::vector<Joint> &joints, int group){
    for (const Joint &J : joints){
        if (par_group(J.index) == group) return true;
    }
    return false;
}

// 并联正解: 组内两个电机的回传(get_motor_data 同坐标, 电机角 - 零偏) -> 两个关节的反馈(API 坐标),
// out[0]/out[1] 对应 kParIndex[group][0]/[1]。与 convert_motor_data 相同: 电机量取反后传入,
// 踝按 (pitch 电机, roll 电机) 传入 motorToJointLeft/Right; 腰的两个电机交叉传入 motorToJointW(14, 13)。
// 时间戳/计数: rx_ns 取两电机较新者(该时刻两个电机的数据都已到齐), rx_count 取较小者(两个电机都有新帧才算一帧新测量)。
struct ParallelSolver {
    ParallelMechanism pm;
    double last_pitch = 0.0;
    double last_roll = 0.0;

    void solve(int group, const PaceJointSample motor_fb[], PaceJointSample out[2]){
        const PaceJointSample &ma = motor_fb[kParIndex[group][0]];
        const PaceJointSample &mb = motor_fb[kParIndex[group][1]];
        double tr, tp, vr, vp, tq_r, tq_p;
        if (group == kWaistGroup){
            std::tie(tr, tp, vr, vp, tq_r, tq_p) = pm.motorToJointWaist(-mb.q, -ma.q, -mb.qd, -ma.qd,
                                                                        -mb.tau, -ma.tau, last_pitch, last_roll);
        }
        else {
            std::tie(tr, tp, vr, vp, tq_r, tq_p) = pm.motorToJointAnkle(group + 1, -ma.q, -mb.q, -ma.qd, -mb.qd,
                                                                        -ma.tau, -mb.tau, last_pitch, last_roll);
        }
        if (!std::isfinite(last_pitch) || !std::isfinite(last_roll)){
            last_pitch = 0.0;  // 迭代发散时下一次从零位重新开始
            last_roll = 0.0;
        }
        PaceJointSample base;
        base.temperature = std::max(ma.temperature, mb.temperature);
        base.error = ma.error | mb.error;
        base.rx_ns = std::max(ma.rx_ns, mb.rx_ns);
        base.rx_count = std::min(ma.rx_count, mb.rx_count);
        base.tx_ns = std::max(ma.tx_ns, mb.tx_ns);
        base.tx_count = std::min(ma.tx_count, mb.tx_count);
        PaceJointSample pitch = base, roll = base;
        pitch.q = (float)tp;
        pitch.qd = (float)vp;
        pitch.tau = (float)tq_p;
        roll.q = (float)tr;
        roll.qd = (float)vr;
        roll.tau = (float)tq_r;
        // 踝: [0] pitch, [1] roll; 腰: [0] 13 roll, [1] 14 pitch
        out[0] = group == kWaistGroup ? roll : pitch;
        out[1] = group == kWaistGroup ? pitch : roll;
    }
};

void print_plan(const Options &o, const std::vector<Joint> &joints){
    printf("[PACE] chirp %.2f->%.2f Hz / %.1f s, rate=%d Hz, move_time=%.1f s\n", o.f0, o.f1, o.duration, o.rate, o.move_time);
    printf("[PACE] %-4s %-28s %-7s %8s %8s %8s %8s\n", "idx", "name", "role", "kp", "kd", "amp", "center");
    for (const Joint &J : joints){
        char center[32] = "q0";
        if (J.excited && !std::isnan(J.center)) snprintf(center, sizeof(center), "%.4f", J.center);
        printf("[PACE] %-4d %-28s %-7s %8.2f %8.3f %8.4f %8s%s\n", J.index, kJointNames[J.index],
               J.excited ? "excite" : "hold", J.kp, J.kd, J.excited ? J.amp : 0.0, J.excited ? center : "q0",
               par_group(J.index) >= 0 ? "  (并联, 关节空间 PD)" : "");
    }
    if (has_group(joints, 0) || has_group(joints, 1)){
        printf("[PACE] 踝电机侧附加阻尼 kd_ff[0] = %.3f\n", o.ankle_kd_ff);
    }
    if (has_group(joints, kWaistGroup)){
        printf("[PACE] 腰电机侧附加阻尼 kd_ff[1] = %.3f\n", o.waist_kd_ff);
    }
    printf("[PACE] 同肢体中不发送指令(无力)的并联关节:");
    const std::vector<int> limp = limp_parallel_joints(joints);
    for (int i : limp) printf(" %s", kJointNames[i]);
    printf("%s\n", limp.empty() ? " 无" : "");
}

int write_plan(const std::string &dir, const Options &o, const std::vector<Joint> &joints){
    const std::string path = dir + "/plan.csv";
    FILE *f = fopen(path.c_str(), "w");
    if (f == nullptr){
        fprintf(stderr, "无法写入 %s\n", path.c_str());
        return 1;
    }
    fprintf(f, "t_s");
    for (const Joint &J : joints){
        if (J.excited) fprintf(f, ",offset_%d", J.index);
    }
    fprintf(f, "\n");
    const double dt = 1.0 / o.rate;
    for (int k = 0; k <= (int)(o.duration * o.rate); ++k){
        const double u = chirp_unit(o, k * dt);
        fprintf(f, "%.6f", k * dt);
        for (const Joint &J : joints){
            if (J.excited) fprintf(f, ",%.7g", J.amp * u);
        }
        fprintf(f, "\n");
    }
    fclose(f);
    printf("[dry-run] 已写入 %s (各被激励关节相对中心位置的偏移)\n", path.c_str());
    return 0;
}

void print_rel_ns(FILE *f, int64_t t_ns, int64_t t0){
    if (t_ns == 0) fprintf(f, ",");
    else fprintf(f, ",%lld", (long long)(t_ns - t0));
}

bool write_ticks(const std::string &dir, const std::vector<Joint> &joints, const std::vector<TickRec> &ticks,
                 const std::vector<JointRec> &recs, const std::vector<int> &motors, const std::vector<MotorRec> &mrecs,
                 size_t n, int64_t t0){
    const std::string path = dir + "/ticks.csv";
    FILE *f = fopen(path.c_str(), "w");
    if (f == nullptr) return false;
    const size_t nj = joints.size();
    fprintf(f, "tick,phase,t_tick_ns,t_read_ns,t_cmd_ns");
    for (const Joint &J : joints){
        const int i = J.index;
        fprintf(f, ",q_des_%d,kp_%d,kd_%d,q_%d,qd_%d,tau_%d,temperature_%d,error_%d,rx_ns_%d,rx_count_%d,tx_ns_%d,tx_count_%d",
                i, i, i, i, i, i, i, i, i, i, i, i);
    }
    for (int m : motors){
        fprintf(f, ",motor_q_%d,motor_qd_%d,motor_tau_%d,motor_rx_ns_%d,motor_rx_count_%d", m, m, m, m, m);
    }
    fprintf(f, "\n");
    for (size_t r = 0; r < n; ++r){
        const TickRec &t = ticks[r];
        fprintf(f, "%u,%s,%lld,%lld,%lld", t.tick, kPhaseName[t.phase],
                (long long)(t.t_tick_ns - t0), (long long)(t.t_read_ns - t0), (long long)(t.t_cmd_ns - t0));
        for (size_t k = 0; k < nj; ++k){
            const JointRec &j = recs[r * nj + k];
            fprintf(f, ",%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%u,%u", j.q_des, j.kp, j.kd, j.q, j.qd, j.tau,
                    (unsigned)j.temperature, (unsigned)j.error);
            print_rel_ns(f, j.rx_ns, t0);
            fprintf(f, ",%u", j.rx_count);
            print_rel_ns(f, j.tx_ns, t0);
            fprintf(f, ",%u", j.tx_count);
        }
        for (size_t k = 0; k < motors.size(); ++k){
            const MotorRec &m = mrecs[r * motors.size() + k];
            fprintf(f, ",%.7g,%.7g,%.7g", m.q, m.qd, m.tau);
            print_rel_ns(f, m.rx_ns, t0);
            fprintf(f, ",%u", m.rx_count);
        }
        fprintf(f, "\n");
    }
    fclose(f);
    return true;
}

}  // namespace

int main(int argc, char **argv){
    Options o;
    if (!parse_args(argc, argv, o)){
        print_usage(argv[0]);
        return 2;
    }
    std::vector<Joint> joints;
    if (!expand_mirror(o)) return 2;
    if (!build_joints(o, joints)) return 2;
    const size_t nj = joints.size();
    print_plan(o, joints);

    if (!o.dry_run && !o.confirm){
        fprintf(stderr, "真机运行必须显式传入 --confirm-suspended(确认已吊装、关节无障碍、急停可用)\n");
        return 2;
    }
    const std::string dir = make_run_dir(o);
    if (dir.empty()) return 1;
    if (o.dry_run) return write_plan(dir, o, joints);

    const int64_t period_ns = 1000000000LL / o.rate;
    const int64_t stale_ns = (int64_t)(o.stale_ms * 1e6);

    // 预分配并触页, 控制循环内不再分配内存
    const double total_s = kProbeMaxS + kEngageS + 2 * o.move_time + o.hold_pre + o.duration + o.hold_post
                         + kReleaseS + kDampS + kZeroS + 1.0;
    const size_t max_ticks = (size_t)(total_s * o.rate) + 16;
    std::vector<TickRec> ticks(max_ticks);
    std::vector<JointRec> recs(max_ticks * nj);
    std::vector<PaceJointSample> fb(nj);
    std::vector<double> q(nj), qd(nj), tau(nj);
    std::vector<int64_t> oldest_rx_ns(nj);  // 并联关节取两个电机中较旧的一帧, 用于超时检查
    std::vector<char> fresh(nj);

    // 并联(踝/腰): 参与的组及其电机(下标即 kParIndex), 两个电机的原始回传都记录下来
    bool group_used[kParGroups] = {false};
    for (const Joint &J : joints){
        if (par_group(J.index) >= 0) group_used[par_group(J.index)] = true;
    }
    std::vector<int> par_motors;
    for (int g = 0; g < kParGroups; ++g){
        if (!group_used[g]) continue;
        par_motors.push_back(kParIndex[g][0]);
        par_motors.push_back(kParIndex[g][1]);
    }
    const size_t nm = par_motors.size();
    std::vector<MotorRec> mrecs(max_ticks * nm);
    PaceJointSample motor_fb[kN] = {};
    PaceJointSample par_fb[kParGroups][2];  // [组][与 kParIndex 相同顺序]
    ParallelSolver par_solver[kParGroups];
    size_t n_ticks = 0;
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0){
        printf("[PACE] 警告: mlockall 失败(%s), 可能出现缺页抖动\n", strerror(errno));
    }

    if (motor_init() != 0){
        fprintf(stderr, "[PACE] motor_init 失败: CAN 总线可能已被 leg_tool / deploy_real 占用\n");
        return 1;
    }
    // 覆盖 motor_init 注册的"直接 exit"处理函数, 保证 Ctrl+C 后仍能走完卸力并落盘
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = on_signal;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);

    MotorCmd cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.mode[0] = 3;  // 力位混控; 不能用 mode 2, 它会把踝/腰并联解算标志永久置 false
    cmd.msg_cmd = 0;  // msg_cmd == 1 会让 1kHz 发送线程逐拍打印调试信息
    for (const Joint &J : joints){
        cmd.motor_id[J.index] = fd_index_2_id(J.index);
    }

    Phase phase = PROBE;
    const int64_t t0 = pace_now_ns();
    int64_t next = t0;
    int64_t phase_start = t0;
    char abort_reason[256] = "";
    bool have_rx_count = false;
    uint32_t overruns = 0;

    auto enter = [&](Phase p, int64_t t){
        printf("[PACE] %.3f s: %s -> %s\n", (t - t0) * 1e-9, kPhaseName[phase], kPhaseName[p]);
        phase = p;
        phase_start = t;
    };

    printf("[PACE] 开始: probe 阶段零刚度, 请确认参与关节处于自然下垂/静止位置\n");
    for (uint32_t tick = 0; phase != DONE && n_ticks < max_ticks; ++tick){
        const int64_t t_tick = next;
        const int64_t t_read = pace_now_ns();
        for (int m : par_motors) pace_get_joint_sample(m, &motor_fb[m]);
        for (int g = 0; g < kParGroups; ++g){
            if (group_used[g]) par_solver[g].solve(g, motor_fb, par_fb[g]);
        }
        for (size_t k = 0; k < nj; ++k){
            const int i = joints[k].index;
            const int g = par_group(i);
            if (g >= 0){
                fb[k] = par_fb[g][i == kParIndex[g][0] ? 0 : 1];
                oldest_rx_ns[k] = std::min(motor_fb[kParIndex[g][0]].rx_ns, motor_fb[kParIndex[g][1]].rx_ns);
            }
            else {
                pace_get_joint_sample(i, &fb[k]);
                oldest_rx_ns[k] = fb[k].rx_ns;
            }
            fresh[k] = have_rx_count && fb[k].rx_count != joints[k].last_rx_count;
            joints[k].last_rx_count = fb[k].rx_count;
            q[k] = api_to_urdf(i, fb[k].q);
            qd[k] = sign_api_to_urdf(i) * fb[k].qd;
            tau[k] = sign_api_to_urdf(i) * fb[k].tau;
        }
        have_rx_count = true;

        // ---------- 安全检查: 只在出力阶段 ----------
        if (phase >= ENGAGE && phase <= RELEASE){
            if (g_stop){
                snprintf(abort_reason, sizeof(abort_reason), "收到 SIGINT/SIGTERM");
            }
            for (size_t k = 0; k < nj && abort_reason[0] == '\0'; ++k){
                Joint &J = joints[k];
                const char *name = kJointNames[J.index];
                if (t_read - oldest_rx_ns[k] > stale_ns){
                    snprintf(abort_reason, sizeof(abort_reason), "%s 反馈 %.1f ms 未更新", name, (t_read - oldest_rx_ns[k]) * 1e-6);
                }
                else if (!std::isfinite(q[k]) || !std::isfinite(qd[k]) || !std::isfinite(tau[k])){
                    snprintf(abort_reason, sizeof(abort_reason), "%s 反馈出现 NaN/Inf", name);
                }
                else if (fb[k].error != 0){
                    snprintf(abort_reason, sizeof(abort_reason), "%s 电机报错 error=0x%02X", name, fb[k].error);
                }
                else if (std::fabs(q[k] - J.last_q_des) > o.max_err){
                    snprintf(abort_reason, sizeof(abort_reason), "%s 跟踪误差 %.4f rad 超过 %.3f", name,
                             q[k] - J.last_q_des, o.max_err);
                }
                else if (fresh[k]){
                    J.vel_bad = std::fabs(qd[k]) > o.max_vel ? J.vel_bad + 1 : 0;
                    J.tau_bad = std::fabs(tau[k]) > o.max_tau ? J.tau_bad + 1 : 0;
                    if (J.vel_bad >= kViolationFrames){
                        snprintf(abort_reason, sizeof(abort_reason), "%s 速度连续 %d 帧超限: %.3f rad/s", name, J.vel_bad, qd[k]);
                    }
                    else if (J.tau_bad >= kViolationFrames){
                        snprintf(abort_reason, sizeof(abort_reason), "%s 力矩连续 %d 帧超限: %.3f Nm", name, J.tau_bad, tau[k]);
                    }
                }
            }
            if (abort_reason[0] != '\0'){
                printf("[PACE] 中止: %s\n", abort_reason);
                enter(DAMP, t_tick);
            }
        }

        // ---------- 阶段切换 ----------
        double tp = (t_tick - phase_start) * 1e-9;
        switch (phase){
            case PROBE: {
                if (g_stop){
                    snprintf(abort_reason, sizeof(abort_reason), "probe 阶段收到停止信号");
                    enter(ZERO, t_tick);
                    break;
                }
                bool all_ready = true;
                for (size_t k = 0; k < nj; ++k){
                    Joint &J = joints[k];
                    if (fresh[k] && tp >= kProbeSettleS){
                        J.probe_sum += q[k];
                        J.probe_min = std::min(J.probe_min, q[k]);
                        J.probe_max = std::max(J.probe_max, q[k]);
                        J.probe_n += 1;
                    }
                    all_ready = all_ready && J.probe_n >= kProbeMinSamples;
                }
                if (tp >= kProbeMinS && all_ready){
                    for (size_t k = 0; k < nj && abort_reason[0] == '\0'; ++k){
                        Joint &J = joints[k];
                        J.q0 = J.probe_sum / J.probe_n;
                        if (std::isnan(J.center)) J.center = J.q0;
                        if (J.probe_max - J.probe_min > kProbeMaxSpread){
                            snprintf(abort_reason, sizeof(abort_reason), "probe 期间 %s 在动(极差 %.4f rad)",
                                     kJointNames[J.index], J.probe_max - J.probe_min);
                        }
                        else if (std::fabs(J.center - J.q0) > o.max_move){
                            snprintf(abort_reason, sizeof(abort_reason), "%s 中心位置 %.4f 距 q0=%.4f 超过 --max-move %.3f",
                                     kJointNames[J.index], J.center, J.q0, o.max_move);
                        }
                    }
                    if (abort_reason[0] != '\0'){
                        printf("[PACE] 中止: %s\n", abort_reason);
                        enter(ZERO, t_tick);
                    }
                    else {
                        for (const Joint &J : joints){
                            printf("[PACE] %-28s q0 = %8.5f  center = %8.5f (%u 帧)\n", kJointNames[J.index], J.q0,
                                   J.excited ? J.center : J.q0, J.probe_n);
                        }
                        enter(ENGAGE, t_tick);
                    }
                }
                else if (tp >= kProbeMaxS){
                    int len = snprintf(abort_reason, sizeof(abort_reason), "probe 超时, 有效反馈不足:");
                    for (const Joint &J : joints){
                        if (J.probe_n < kProbeMinSamples && len < (int)sizeof(abort_reason)){
                            len += snprintf(abort_reason + len, sizeof(abort_reason) - len, " %s(%u帧)",
                                            kJointNames[J.index], J.probe_n);
                        }
                    }
                    printf("[PACE] 中止: %s\n", abort_reason);
                    enter(ZERO, t_tick);
                }
                break;
            }
            case ENGAGE:    if (tp >= kEngageS) enter(MOVE_IN, t_tick); break;
            case MOVE_IN:   if (tp >= o.move_time) enter(HOLD_PRE, t_tick); break;
            case HOLD_PRE:  if (tp >= o.hold_pre) enter(CHIRP, t_tick); break;
            case CHIRP:     if (tp >= o.duration) enter(HOLD_POST, t_tick); break;
            case HOLD_POST: if (tp >= o.hold_post) enter(MOVE_OUT, t_tick); break;
            case MOVE_OUT:  if (tp >= o.move_time) enter(RELEASE, t_tick); break;
            case RELEASE:   if (tp >= kReleaseS) enter(ZERO, t_tick); break;
            case DAMP:      if (tp >= kDampS) enter(ZERO, t_tick); break;
            case ZERO:      if (tp >= kZeroS) enter(DONE, t_tick); break;
            case DONE:      break;
        }
        tp = (t_tick - phase_start) * 1e-9;

        // ---------- 本拍指令 ----------
        const double u = phase == CHIRP ? chirp_unit(o, tp) : 0.0;
        TickRec &tr = ticks[n_ticks];
        for (size_t k = 0; k < nj; ++k){
            Joint &J = joints[k];
            const double center = J.excited ? J.center : J.q0;
            double q_des = q[k], kp = 0.0, kd = 0.0;
            switch (phase){
                case PROBE:     kd = J.kd; break;  // 零刚度 + 阻尼: 只读位置, 防止关节意外下落过快
                case ENGAGE:    q_des = J.q0; kp = J.kp * smooth01(tp / kEngageS); kd = J.kd; break;
                case MOVE_IN:   q_des = J.q0 + (center - J.q0) * smooth01(tp / o.move_time); kp = J.kp; kd = J.kd; break;
                case HOLD_PRE:
                case HOLD_POST: q_des = center; kp = J.kp; kd = J.kd; break;
                case CHIRP:     q_des = center + (J.excited ? J.amp * u : 0.0); kp = J.kp; kd = J.kd; break;
                case MOVE_OUT:  q_des = center + (J.q0 - center) * smooth01(tp / o.move_time); kp = J.kp; kd = J.kd; break;
                case RELEASE:   q_des = J.q0; kp = J.kp * (1 - smooth01(tp / kReleaseS)); kd = J.kd; break;
                case DAMP:      kd = J.kd; break;  // 纯阻尼, 不注入能量
                case ZERO:
                case DONE:      break;
            }
            const int i = J.index;
            cmd.q_des[i] = (float)urdf_to_api(i, q_des);
            cmd.qd_des[i] = 0.0f;
            cmd.kp[i] = (float)kp;
            cmd.kd[i] = (float)kd;
            cmd.tff[i] = 0.0f;
            J.last_q_des = q_des;

            JointRec &r = recs[n_ticks * nj + k];
            r.q_des = (float)q_des;
            r.kp = (float)kp;
            r.kd = (float)kd;
            r.q = (float)q[k];
            r.qd = (float)qd[k];
            r.tau = (float)tau[k];
            r.rx_ns = fb[k].rx_ns;
            r.tx_ns = fb[k].tx_ns;
            r.rx_count = fb[k].rx_count;
            r.tx_count = fb[k].tx_count;
            r.temperature = fb[k].temperature;
            r.error = fb[k].error;
        }
        for (size_t k = 0; k < nm; ++k){
            const PaceJointSample &s = motor_fb[par_motors[k]];
            MotorRec &m = mrecs[n_ticks * nm + k];
            m.q = s.q;
            m.qd = s.qd;
            m.tau = s.tau;
            m.rx_ns = s.rx_ns;
            m.rx_count = s.rx_count;
        }
        // 踝/腰电机侧附加阻尼, ZERO 之后撤掉, 保证最后一帧是零力矩
        cmd.kd_ff[0] = phase < ZERO ? (float)o.ankle_kd_ff : 0.0f;
        cmd.kd_ff[1] = phase < ZERO ? (float)o.waist_kd_ff : 0.0f;
        sendMotorCmd(&cmd);
        tr.t_tick_ns = t_tick;
        tr.t_read_ns = t_read;
        tr.t_cmd_ns = pace_now_ns();
        tr.tick = tick;
        tr.phase = phase;
        n_ticks += 1;

        next += period_ns;
        const int64_t now = pace_now_ns();
        if (now > next){
            overruns += 1;
            if (now > next + period_ns) next = now;  // 严重超时时重新对齐, 不追赶补拍
        }
        sleep_until_ns(next);
    }

    // motor_id 置 0 会清掉这些电机的发送槽, 之后不再给它们发报文(最后一帧是零力矩指令)
    for (const Joint &J : joints){
        cmd.kp[J.index] = 0.0f;
        cmd.kd[J.index] = 0.0f;
        cmd.tff[J.index] = 0.0f;
        cmd.motor_id[J.index] = 0;
    }
    cmd.kd_ff[0] = 0.0f;
    cmd.kd_ff[1] = 0.0f;
    sendMotorCmd(&cmd);
    usleep(20 * 1000);

    // ---------- 逐关节统计(chirp 段) ----------
    std::vector<uint32_t> frames(nj, 0);
    std::vector<double> fb_hz(nj, 0.0), max_gap_ms(nj, 0.0), max_track(nj, 0.0);
    for (size_t k = 0; k < nj; ++k){
        int64_t first_rx = 0, prev_rx = 0, last_rx = 0;
        for (size_t r = 1; r < n_ticks; ++r){
            const JointRec &cur = recs[r * nj + k];
            if (ticks[r].phase != CHIRP || cur.rx_count == recs[(r - 1) * nj + k].rx_count) continue;
            if (prev_rx != 0) max_gap_ms[k] = std::max(max_gap_ms[k], (cur.rx_ns - prev_rx) * 1e-6);
            if (first_rx == 0) first_rx = cur.rx_ns;
            prev_rx = cur.rx_ns;
            last_rx = cur.rx_ns;
            frames[k] += 1;
            max_track[k] = std::max(max_track[k], (double)std::fabs(cur.q_des - cur.q));
        }
        if (frames[k] > 1 && last_rx > first_rx) fb_hz[k] = (frames[k] - 1) / ((last_rx - first_rx) * 1e-9);
    }

    // 参与关节所在 CAN 通道上, 发送线程取到空队列(并改为重发上一份快照)的次数, 整个运行期间累计
    bool channel_used[CHANNEL_NUMBER] = {false};
    for (const Joint &J : joints) channel_used[fd_index_2_channel(J.index)] = true;
    uint64_t tx_empty[CHANNEL_NUMBER] = {0}, tx_reused[CHANNEL_NUMBER] = {0};
    for (int c = 0; c < CHANNEL_NUMBER; ++c) pace_get_tx_queue_stats(c, &tx_empty[c], &tx_reused[c]);

    // ---------- 落盘 ----------
    const bool ok_csv = write_ticks(dir, joints, ticks, recs, par_motors, mrecs, n_ticks, t0);
    const std::string meta_path = dir + "/meta.json";
    FILE *mf = fopen(meta_path.c_str(), "w");
    const bool ok_meta = mf != nullptr;
    if (ok_meta){
        fprintf(mf, "{\n");
        fprintf(mf, "  \"schema\": \"pace_chirp/1\",\n");
        fprintf(mf, "  \"angle_frame\": \"urdf: q_urdf = (q_api * motor_direction + pos_offset) * dance_dir, leg_offset = 0\",\n");
        fprintf(mf, "  \"time_base\": \"CLOCK_MONOTONIC ns, relative to t0\",\n");
        fprintf(mf, "  \"t0_monotonic_ns\": %lld,\n", (long long)t0);
        fprintf(mf, "  \"gains_file\": \"%s\",\n", json_escape(o.gains_path).c_str());
        fprintf(mf, "  \"mirror\": \"%s\",\n", o.mirror.c_str());
        fprintf(mf, "  \"hold\": \"%s\",\n", json_escape(o.hold).c_str());
        fprintf(mf, "  \"joints\": [\n");
        for (size_t k = 0; k < nj; ++k){
            const Joint &J = joints[k];
            const int i = J.index;
            fprintf(mf, "    {\"index\": %d, \"name\": \"%s\", \"motor_id\": %d, \"parallel\": %s, \"limb\": \"%s\", \"role\": \"%s\", "
                        "\"kp\": %.6g, \"kd\": %.6g, \"amp\": %.6g, \"center\": %.6f, \"q0\": %.6f, "
                        "\"motor_direction\": %d, \"dance_dir\": %d, \"pos_offset\": %.4f, \"lf1_zero_offset_rad\": %.6f, "
                        "\"chirp_feedback_frames\": %u, \"chirp_feedback_hz\": %.2f, \"max_rx_gap_ms\": %.3f, \"max_track_err\": %.5f}%s\n",
                    i, kJointNames[i], fd_index_2_id(i), par_group(i) >= 0 ? "true" : "false", kLimbName[limb_of(i)],
                    J.excited ? "excite" : "hold",
                    J.kp, J.kd, J.excited ? J.amp : 0.0, J.excited ? J.center : J.q0, J.q0,
                    kMotorDirection[i], kDanceDir[i], kPosOffset[i], lf1_zero_offset_rad[i],
                    frames[k], fb_hz[k], max_gap_ms[k], max_track[k], k + 1 < nj ? "," : "");
        }
        fprintf(mf, "  ],\n");
        fprintf(mf, "  \"limp_parallel_joints\": [");
        const std::vector<int> limp = limp_parallel_joints(joints);
        for (size_t k = 0; k < limp.size(); ++k){
            fprintf(mf, "%s\"%s\"", k ? ", " : "", kJointNames[limp[k]]);
        }
        fprintf(mf, "],\n");
        // 并联组的电机列表: 踝(组 0/1)与腰(组 2)分开写
        auto print_motors = [&](bool waist){
            bool first = true;
            for (int m : par_motors){
                if ((par_group(m) == kWaistGroup) != waist) continue;
                fprintf(mf, "%s{\"index\": %d, \"motor_id\": %d}", first ? "" : ", ", m, fd_index_2_id(m));
                first = false;
            }
        };
        fprintf(mf, "  \"ankle\": {\"kd_ff\": %.6g, \"motors\": [", o.ankle_kd_ff);
        print_motors(false);
        fprintf(mf, "], \"joint_tau_limit\": {\"pitch\": 60, \"roll\": 20}, \"motor_tau_limit\": 25,\n"
                    "            \"note\": \"joint-space PD computed in C tx thread from solved joint state; motor kd = kd_ff; motor_* columns = raw motor feedback (get_motor_data frame, before parallel solve)\"},\n");
        fprintf(mf, "  \"waist\": {\"kd_ff\": %.6g, \"motors\": [", o.waist_kd_ff);
        print_motors(true);
        fprintf(mf, "], \"joint_tau_limit\": {\"roll\": 60, \"pitch\": 60}, \"motor_tau_limit\": 50,\n"
                    "            \"note\": \"waist roll/pitch (13/14) hold only; joint-space PD as for the ankles, motor kd = kd_ff[1]\"},\n");
        fprintf(mf, "  \"chirp\": {\"type\": \"linear\", \"f0\": %.6f, \"f1\": %.6f, \"duration\": %.6f, \"ramp\": %.6f},\n",
                o.f0, o.f1, o.duration, o.ramp);
        fprintf(mf, "  \"move_time\": %.3f,\n  \"hold_pre\": %.3f,\n  \"hold_post\": %.3f,\n  \"rate_hz\": %d,\n",
                o.move_time, o.hold_pre, o.hold_post, o.rate);
        fprintf(mf, "  \"qd_des\": 0.0,\n  \"tau_ff\": 0.0,\n");
        fprintf(mf, "  \"safety\": {\"max_vel\": %.3f, \"max_tau\": %.3f, \"max_err\": %.3f, \"stale_ms\": %.1f},\n",
                o.max_vel, o.max_tau, o.max_err, o.stale_ms);
        fprintf(mf, "  \"ticks\": %zu,\n  \"overruns\": %u,\n", n_ticks, overruns);
        fprintf(mf, "  \"tx_queue\": [");
        bool first_channel = true;
        for (int c = 0; c < CHANNEL_NUMBER; ++c){
            if (!channel_used[c]) continue;
            fprintf(mf, "%s{\"channel\": %d, \"empty\": %llu, \"reused\": %llu}", first_channel ? "" : ", ", c,
                    (unsigned long long)tx_empty[c], (unsigned long long)tx_reused[c]);
            first_channel = false;
        }
        fprintf(mf, "],\n");
        fprintf(mf, "  \"completed\": %s,\n  \"abort_reason\": \"%s\"\n", abort_reason[0] == '\0' ? "true" : "false",
                json_escape(abort_reason).c_str());
        fprintf(mf, "}\n");
        fclose(mf);
    }

    printf("[PACE] %s, 共 %zu 拍, 超时 %u 拍\n", abort_reason[0] == '\0' ? "采集完成" : "已中止", n_ticks, overruns);
    for (size_t k = 0; k < nj; ++k){
        printf("[PACE] %-28s chirp 段反馈 %u 帧 约 %.1f Hz, 最大间隔 %.2f ms, 最大 |q_des-q| %.4f rad\n",
               kJointNames[joints[k].index], frames[k], fb_hz[k], max_gap_ms[k], max_track[k]);
    }
    for (int c = 0; c < CHANNEL_NUMBER; ++c){
        if (!channel_used[c]) continue;
        printf("[PACE] CAN 通道 %d: 发送队列为空 %llu 次, 其中重发上一份快照 %llu 次\n", c,
               (unsigned long long)tx_empty[c], (unsigned long long)tx_reused[c]);
    }
    printf("[PACE] 输出: %s/ticks.csv%s, meta.json%s\n", dir.c_str(), ok_csv ? "" : "(写入失败!)", ok_meta ? "" : "(写入失败!)");

    fflush(stdout);
    close_canDevice();
    // 库里的收发线程是 detach 的, 正常 return 会在线程仍运行时析构全局对象, 用 _exit 直接退出
    _exit(abort_reason[0] == '\0' ? 0 : 3);
}

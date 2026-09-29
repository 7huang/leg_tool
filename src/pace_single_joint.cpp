/*
 * pace_single_joint: PACE 最小实验 —— 单关节 chirp 激励, 同时记录下发指令与电机回传。
 *
 * 前提: 机器人躯干刚性吊装、被测关节活动范围内无障碍、实体急停可触达;
 *       不能与 leg_tool / deploy_real 同时运行(motor_init 的 flock 锁会拒绝)。
 * 只支持串联关节: 并联的踝(4/5/10/11)与腰(13/14)在 C 端做关节空间 PD + 解算, 另行处理。
 * 除被测关节外其余 motor_id 置 0, 本程序不会向其它电机发送任何报文。
 *
 * 阶段: probe(零刚度+阻尼, 读初始位置 q0) -> engage(kp 平滑升到目标) -> hold_pre
 *       -> chirp(q0 + 线性扫频) -> hold_post -> release(kp 平滑降到 0) -> zero -> done
 *       任何安全检查失败或 Ctrl+C: 立即进入 damp(kp=0, 只留阻尼, 不注入能量) -> zero -> done
 *
 * 时间戳: 全部为 CLOCK_MONOTONIC, CSV 中为相对 t0(第一拍)的纳秒
 *   t_cmd_ns : 本拍指令写入 motor_cmd_upper 的时刻, 等价于部署时 Python 调 set_motor_cmd 的时刻。
 *              之后 1kHz 打包线程 + 1kHz 发送线程 + tiktak 4 拍轮询带来的延迟属于执行器延迟, 交给 PACE 辨识。
 *   rx_ns    : 反馈帧被 motorState_fd 解包的时刻(比上总线晚 <~0.5ms)
 *   tx_ns    : 最近一次把该电机报文 write() 进 socket 的时刻(诊断用)
 *
 * 输出: <out>/<日期_时间>_j<index>/ticks.csv 与 meta.json
 *   指令序列: 每行 (t_cmd_ns, q_des, kp, kd)
 *   测量序列: 取 rx_count 相对上一行变化的行 (rx_ns, q, qd, tau), 每个新反馈帧恰好一条
 *   角度坐标系与 get_motor_data 相同: 电机角 - lf1_zero_offset_rad[index],
 *   不含 low_control.py 的 motor_direction / leg_offset, 转 URDF 在离线处理时做。
 */
#include "ec_api_without_ros.hpp"
#include "pace_stamp.h"

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
#include <vector>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace {

enum Phase : int { PROBE = 0, ENGAGE, HOLD_PRE, CHIRP, HOLD_POST, RELEASE, DAMP, ZERO, DONE };
const char *kPhaseName[] = {"probe", "engage", "hold_pre", "chirp", "hold_post", "release", "damp", "zero", "done"};

constexpr double kProbeMinS = 0.5;         // 探测阶段最短时长 [s]
constexpr double kProbeMaxS = 3.0;         // 探测阶段超时 [s]
constexpr double kProbeSettleS = 0.2;      // 探测开始后丢弃的反馈时长 [s]
constexpr uint32_t kProbeMinSamples = 50;  // 计算 q0 至少需要的新反馈帧数
constexpr double kProbeMaxSpread = 0.02;   // 探测期关节应静止, q 极差上限 [rad]
constexpr double kEngageS = 0.5;
constexpr double kReleaseS = 0.5;
constexpr double kDampS = 0.5;
constexpr double kZeroS = 0.1;
constexpr int kViolationFrames = 3;        // 速度/力矩连续超限的反馈帧数才中止, 过滤单帧毛刺

struct Options {
    int joint = -1;
    std::string name;
    double kp = -1.0;
    double kd = -1.0;
    double amp = 0.05;
    double f0 = 0.1;
    double f1 = 2.0;
    double duration = 20.0;
    double ramp = 1.0;
    double hold_pre = 1.0;
    double hold_post = 1.0;
    int rate = 1000;
    double max_vel = 5.0;
    double max_tau = 10.0;
    double pos_margin = 0.1;
    double stale_ms = 50.0;
    std::string out = "./pace_log";
    bool confirm = false;
    bool dry_run = false;
};

struct Row {
    int64_t t_tick_ns;
    int64_t t_read_ns;
    int64_t t_cmd_ns;
    uint32_t tick;
    int32_t phase;
    float q_des;
    float qd_des;
    float kp;
    float kd;
    float tau_ff;
    PaceJointSample fb;
};

std::atomic<bool> g_stop{false};

void on_signal(int){
    g_stop = true;
}

bool is_parallel_joint(int j){
    return j == 4 || j == 5 || j == 10 || j == 11 || j == 13 || j == 14;
}

void print_usage(const char *prog){
    printf("用法: sudo %s --joint <index> --kp <Nm/rad> --kd <Nms/rad> [选项] --confirm-suspended\n"
           "  --joint N          关节下标 0-28(与 MotorCmd 数组下标一致), 不支持 4/5/10/11/13/14\n"
           "  --name S           关节名, 仅写入 meta.json\n"
           "  --kp X --kd X      PD 增益, 必须与仿真/部署一致\n"
           "  --amp X            chirp 振幅 [rad], 默认 0.05, 上限 0.3\n"
           "  --f0 X --f1 X      起止频率 [Hz], 默认 0.1 -> 2.0, 上限 10\n"
           "  --duration X       chirp 时长 [s], 默认 20\n"
           "  --ramp X           chirp 首尾余弦淡入淡出 [s], 默认 1.0\n"
           "  --hold-pre X       chirp 前保持 [s], 默认 1.0\n"
           "  --hold-post X      chirp 后保持 [s], 默认 1.0\n"
           "  --rate N           控制/记录频率 [Hz], 默认 1000, 范围 100-1000\n"
           "  --max-vel X        速度安全阈值 [rad/s], 默认 5\n"
           "  --max-tau X        力矩安全阈值 [Nm], 默认 10\n"
           "  --pos-margin X     位置超出 q0 +/- (amp + margin) 即中止 [rad], 默认 0.1\n"
           "  --stale-ms X       反馈超过该时长未更新即中止 [ms], 默认 50\n"
           "  --out DIR          输出根目录, 默认 ./pace_log\n"
           "  --dry-run          不连电机, 只生成 plan.csv 检查波形\n"
           "  --confirm-suspended 确认已吊装、急停可用(真机运行必需)\n", prog);
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
        if      (key == "--joint")      o.joint = atoi(v);
        else if (key == "--name")       o.name = v;
        else if (key == "--kp")         o.kp = strtod(v, nullptr);
        else if (key == "--kd")         o.kd = strtod(v, nullptr);
        else if (key == "--amp")        o.amp = strtod(v, nullptr);
        else if (key == "--f0")         o.f0 = strtod(v, nullptr);
        else if (key == "--f1")         o.f1 = strtod(v, nullptr);
        else if (key == "--duration")   o.duration = strtod(v, nullptr);
        else if (key == "--ramp")       o.ramp = strtod(v, nullptr);
        else if (key == "--hold-pre")   o.hold_pre = strtod(v, nullptr);
        else if (key == "--hold-post")  o.hold_post = strtod(v, nullptr);
        else if (key == "--rate")       o.rate = atoi(v);
        else if (key == "--max-vel")    o.max_vel = strtod(v, nullptr);
        else if (key == "--max-tau")    o.max_tau = strtod(v, nullptr);
        else if (key == "--pos-margin") o.pos_margin = strtod(v, nullptr);
        else if (key == "--stale-ms")   o.stale_ms = strtod(v, nullptr);
        else if (key == "--out")        o.out = v;
        else {
            fprintf(stderr, "未知参数: %s\n", key.c_str());
            return false;
        }
    }
    return o.joint >= 0;
}

bool validate(const Options &o){
    auto fail = [](const char *msg){
        fprintf(stderr, "参数错误: %s\n", msg);
        return false;
    };
    if (o.joint < 0 || o.joint >= motor_number_) return fail("--joint 必须在 [0, 28]");
    if (is_parallel_joint(o.joint)) return fail("并联关节 4/5/10/11/13/14 不在最小实验范围内");
    if (!(o.kp > 0 && o.kp <= 200)) return fail("--kp 必须在 (0, 200]");
    if (!(o.kd >= 0 && o.kd <= 10)) return fail("--kd 必须在 [0, 10]");
    if (!(o.amp > 0 && o.amp <= 0.3)) return fail("--amp 必须在 (0, 0.3] rad");
    if (!(o.f0 > 0 && o.f0 < o.f1 && o.f1 <= 10)) return fail("频率必须满足 0 < f0 < f1 <= 10 Hz");
    if (!(o.duration >= 2 && o.duration <= 120)) return fail("--duration 必须在 [2, 120] s");
    if (!(o.ramp >= 0.1 && 2 * o.ramp <= o.duration)) return fail("--ramp 必须 >= 0.1 s 且不超过 duration/2");
    if (!(o.hold_pre >= 0 && o.hold_pre <= 10 && o.hold_post >= 0 && o.hold_post <= 10)) return fail("--hold-pre/--hold-post 必须在 [0, 10] s");
    if (o.rate < 100 || o.rate > 1000) return fail("--rate 必须在 [100, 1000] Hz");
    if (!(o.max_vel > 0 && o.max_tau > 0 && o.pos_margin > 0 && o.stale_ms >= 10)) return fail("安全阈值必须为正, --stale-ms >= 10");
    if (o.amp * 2 * M_PI * o.f1 > 0.8 * o.max_vel) return fail("参考峰值速度 amp*2*pi*f1 超过 0.8*max-vel, 请减小 amp/f1");
    return true;
}

// 线性扫频, 首尾 ramp 秒余弦包络淡入淡出, 保证目标位置和速度在起止处连续
double chirp_offset(const Options &o, double t){
    if (t <= 0 || t >= o.duration) return 0.0;
    double env = 1.0;
    if (t < o.ramp){
        env = 0.5 * (1 - cos(M_PI * t / o.ramp));
    }
    else if (t > o.duration - o.ramp){
        env = 0.5 * (1 - cos(M_PI * (o.duration - t) / o.ramp));
    }
    const double phase = 2 * M_PI * (o.f0 * t + 0.5 * (o.f1 - o.f0) / o.duration * t * t);
    return o.amp * env * sin(phase);
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
    const std::string dir = o.out + "/" + stamp + "_j" + std::to_string(o.joint) + (o.dry_run ? "_dry" : "");
    if (!mkdir_p(dir)){
        fprintf(stderr, "创建输出目录失败: %s (%s)\n", dir.c_str(), strerror(errno));
        return "";
    }
    return dir;
}

int write_plan(const std::string &dir, const Options &o){
    const std::string path = dir + "/plan.csv";
    FILE *f = fopen(path.c_str(), "w");
    if (f == nullptr){
        fprintf(stderr, "无法写入 %s\n", path.c_str());
        return 1;
    }
    fprintf(f, "t_s,q_des_offset\n");
    const double dt = 1.0 / o.rate;
    double max_off = 0, max_vel = 0, prev = 0;
    for (int k = 0; k <= (int)(o.duration * o.rate); ++k){
        const double t = k * dt;
        const double off = chirp_offset(o, t);
        if (k > 0) max_vel = std::max(max_vel, std::fabs(off - prev) / dt);
        max_off = std::max(max_off, std::fabs(off));
        prev = off;
        fprintf(f, "%.6f,%.7g\n", t, off);
    }
    fclose(f);
    printf("[dry-run] 已写入 %s\n[dry-run] 最大偏移 %.4f rad, 最大参考速度 %.3f rad/s\n", path.c_str(), max_off, max_vel);
    return 0;
}

// 相对 t0 输出; 0 表示从未收到/发出过, 输出为空
void print_rel_ns(FILE *f, int64_t t_ns, int64_t t0){
    if (t_ns == 0) fprintf(f, ",");
    else fprintf(f, ",%lld", (long long)(t_ns - t0));
}

bool write_ticks(const std::string &dir, const std::vector<Row> &rows, size_t n, int64_t t0){
    const std::string path = dir + "/ticks.csv";
    FILE *f = fopen(path.c_str(), "w");
    if (f == nullptr) return false;
    fprintf(f, "tick,phase,t_tick_ns,t_read_ns,t_cmd_ns,q_des,qd_des,kp,kd,tau_ff,"
               "q,qd,tau,temperature,error,rx_ns,rx_count,tx_ns,tx_count\n");
    for (size_t i = 0; i < n; ++i){
        const Row &r = rows[i];
        fprintf(f, "%u,%s,%lld,%lld,%lld,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%u,%u",
                r.tick, kPhaseName[r.phase],
                (long long)(r.t_tick_ns - t0), (long long)(r.t_read_ns - t0), (long long)(r.t_cmd_ns - t0),
                r.q_des, r.qd_des, r.kp, r.kd, r.tau_ff,
                r.fb.q, r.fb.qd, r.fb.tau, (unsigned)r.fb.temperature, (unsigned)r.fb.error);
        print_rel_ns(f, r.fb.rx_ns, t0);
        fprintf(f, ",%u", r.fb.rx_count);
        print_rel_ns(f, r.fb.tx_ns, t0);
        fprintf(f, ",%u\n", r.fb.tx_count);
    }
    fclose(f);
    return true;
}

std::string json_escape(const std::string &s){
    std::string out;
    for (char c : s){
        if (c == '"' || c == '\\') out += '\\';
        out += c;
    }
    return out;
}

}  // namespace

int main(int argc, char **argv){
    Options o;
    if (!parse_args(argc, argv, o)){
        print_usage(argv[0]);
        return 2;
    }
    if (!validate(o)) return 2;

    const int j = o.joint;
    const int motor_id = fd_index_2_id(j);
    const int64_t period_ns = 1000000000LL / o.rate;
    const int64_t stale_ns = (int64_t)(o.stale_ms * 1e6);

    printf("[PACE] joint index=%d motor_id=%d name=%s\n", j, motor_id, o.name.empty() ? "-" : o.name.c_str());
    printf("[PACE] kp=%.3f kd=%.3f amp=%.3f rad chirp %.2f->%.2f Hz / %.1f s, rate=%d Hz\n",
           o.kp, o.kd, o.amp, o.f0, o.f1, o.duration, o.rate);
    printf("[PACE] 参考峰值速度约 %.3f rad/s, 安全阈值 vel=%.2f rad/s tau=%.2f Nm\n",
           o.amp * 2 * M_PI * o.f1, o.max_vel, o.max_tau);

    if (!o.dry_run && !o.confirm){
        fprintf(stderr, "真机运行必须显式传入 --confirm-suspended(确认已吊装、关节无障碍、急停可用)\n");
        return 2;
    }
    const std::string dir = make_run_dir(o);
    if (dir.empty()) return 1;
    if (o.dry_run) return write_plan(dir, o);

    // 预分配并触页, 控制循环内不再分配内存
    const double total_s = kProbeMaxS + kEngageS + o.hold_pre + o.duration + o.hold_post
                         + kReleaseS + kDampS + kZeroS + 1.0;
    std::vector<Row> rows((size_t)(total_s * o.rate) + 16);
    size_t n_rows = 0;
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
    cmd.motor_id[j] = motor_id;

    Phase phase = PROBE;
    const int64_t t0 = pace_now_ns();
    int64_t next = t0;
    int64_t phase_start = t0;
    char abort_reason[160] = "";
    double q0 = 0.0;
    uint32_t last_rx_count = 0;
    bool have_rx_count = false;
    double probe_sum = 0, probe_min = 1e9, probe_max = -1e9;
    uint32_t probe_n = 0;
    int vel_bad = 0, tau_bad = 0;
    uint32_t overruns = 0;

    auto enter = [&](Phase p, int64_t t){
        printf("[PACE] %.3f s: %s -> %s\n", (t - t0) * 1e-9, kPhaseName[phase], kPhaseName[p]);
        phase = p;
        phase_start = t;
    };

    printf("[PACE] 开始: probe 阶段零刚度, 请确认关节处于自然下垂/静止位置\n");
    for (uint32_t tick = 0; phase != DONE && n_rows < rows.size(); ++tick){
        const int64_t t_tick = next;
        const int64_t t_read = pace_now_ns();
        PaceJointSample fb;
        pace_get_joint_sample(j, &fb);
        const bool fresh = have_rx_count && fb.rx_count != last_rx_count;
        last_rx_count = fb.rx_count;
        have_rx_count = true;

        // ---------- 安全检查: 只在出力阶段 ----------
        if (phase >= ENGAGE && phase <= RELEASE){
            if (g_stop){
                snprintf(abort_reason, sizeof(abort_reason), "收到 SIGINT/SIGTERM");
            }
            else if (t_read - fb.rx_ns > stale_ns){
                snprintf(abort_reason, sizeof(abort_reason), "反馈 %.1f ms 未更新", (t_read - fb.rx_ns) * 1e-6);
            }
            else if (!std::isfinite(fb.q) || !std::isfinite(fb.qd) || !std::isfinite(fb.tau)){
                snprintf(abort_reason, sizeof(abort_reason), "反馈出现 NaN/Inf");
            }
            else if (fb.error != 0){
                snprintf(abort_reason, sizeof(abort_reason), "电机报错 error=0x%02X", fb.error);
            }
            else if (std::fabs(fb.q - q0) > o.amp + o.pos_margin){
                snprintf(abort_reason, sizeof(abort_reason), "位置 %.4f 超出 q0=%.4f +/- %.3f", fb.q, q0, o.amp + o.pos_margin);
            }
            else if (fresh){
                vel_bad = std::fabs(fb.qd) > o.max_vel ? vel_bad + 1 : 0;
                tau_bad = std::fabs(fb.tau) > o.max_tau ? tau_bad + 1 : 0;
                if (vel_bad >= kViolationFrames){
                    snprintf(abort_reason, sizeof(abort_reason), "速度连续 %d 帧超限: %.3f rad/s", vel_bad, fb.qd);
                }
                else if (tau_bad >= kViolationFrames){
                    snprintf(abort_reason, sizeof(abort_reason), "力矩连续 %d 帧超限: %.3f Nm", tau_bad, fb.tau);
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
            case PROBE:
                if (g_stop){
                    snprintf(abort_reason, sizeof(abort_reason), "probe 阶段收到停止信号");
                    enter(ZERO, t_tick);
                    break;
                }
                if (fresh && tp >= kProbeSettleS){
                    probe_sum += fb.q;
                    probe_min = std::min(probe_min, (double)fb.q);
                    probe_max = std::max(probe_max, (double)fb.q);
                    probe_n += 1;
                }
                if (tp >= kProbeMinS && probe_n >= kProbeMinSamples){
                    if (probe_max - probe_min > kProbeMaxSpread){
                        snprintf(abort_reason, sizeof(abort_reason), "probe 期间关节在动(极差 %.4f rad)", probe_max - probe_min);
                        printf("[PACE] 中止: %s\n", abort_reason);
                        enter(ZERO, t_tick);
                    }
                    else {
                        q0 = probe_sum / probe_n;
                        printf("[PACE] q0 = %.5f rad (%u 帧)\n", q0, probe_n);
                        enter(ENGAGE, t_tick);
                    }
                }
                else if (tp >= kProbeMaxS){
                    snprintf(abort_reason, sizeof(abort_reason), "probe 超时: %.1f s 内只收到 %u 帧有效反馈", kProbeMaxS, probe_n);
                    printf("[PACE] 中止: %s\n", abort_reason);
                    enter(ZERO, t_tick);
                }
                break;
            case ENGAGE:    if (tp >= kEngageS) enter(HOLD_PRE, t_tick); break;
            case HOLD_PRE:  if (tp >= o.hold_pre) enter(CHIRP, t_tick); break;
            case CHIRP:     if (tp >= o.duration) enter(HOLD_POST, t_tick); break;
            case HOLD_POST: if (tp >= o.hold_post) enter(RELEASE, t_tick); break;
            case RELEASE:   if (tp >= kReleaseS) enter(ZERO, t_tick); break;
            case DAMP:      if (tp >= kDampS) enter(ZERO, t_tick); break;
            case ZERO:      if (tp >= kZeroS) enter(DONE, t_tick); break;
            case DONE:      break;
        }
        tp = (t_tick - phase_start) * 1e-9;

        // ---------- 本拍指令 ----------
        double q_des = fb.q, kp = 0.0, kd = 0.0;
        switch (phase){
            case PROBE:     kd = o.kd; break;  // 零刚度 + 阻尼: 只读位置, 防止关节意外下落过快
            case ENGAGE:    q_des = q0; kp = o.kp * smooth01(tp / kEngageS); kd = o.kd; break;
            case HOLD_PRE:
            case HOLD_POST: q_des = q0; kp = o.kp; kd = o.kd; break;
            case CHIRP:     q_des = q0 + chirp_offset(o, tp); kp = o.kp; kd = o.kd; break;
            case RELEASE:   q_des = q0; kp = o.kp * (1 - smooth01(tp / kReleaseS)); kd = o.kd; break;
            case DAMP:      kd = o.kd; break;  // 纯阻尼, 不注入能量
            case ZERO:
            case DONE:      break;
        }
        cmd.q_des[j] = (float)q_des;
        cmd.qd_des[j] = 0.0f;
        cmd.kp[j] = (float)kp;
        cmd.kd[j] = (float)kd;
        cmd.tff[j] = 0.0f;
        sendMotorCmd(&cmd);
        const int64_t t_cmd = pace_now_ns();

        Row &r = rows[n_rows++];
        r.t_tick_ns = t_tick;
        r.t_read_ns = t_read;
        r.t_cmd_ns = t_cmd;
        r.tick = tick;
        r.phase = phase;
        r.q_des = cmd.q_des[j];
        r.qd_des = cmd.qd_des[j];
        r.kp = cmd.kp[j];
        r.kd = cmd.kd[j];
        r.tau_ff = cmd.tff[j];
        r.fb = fb;

        next += period_ns;
        const int64_t now = pace_now_ns();
        if (now > next){
            overruns += 1;
            if (now > next + period_ns) next = now;  // 严重超时时重新对齐, 不追赶补拍
        }
        sleep_until_ns(next);
    }

    // motor_id 置 0 会清掉该电机的发送槽, 之后不再给它发报文(最后一帧是零力矩指令)
    cmd.kp[j] = 0.0f;
    cmd.kd[j] = 0.0f;
    cmd.tff[j] = 0.0f;
    cmd.motor_id[j] = 0;
    sendMotorCmd(&cmd);
    usleep(20 * 1000);

    // ---------- 统计 ----------
    uint32_t chirp_frames = 0;
    double chirp_first_rx = 0, chirp_last_rx = 0, max_rx_gap_ms = 0, max_track_err = 0;
    int64_t prev_rx = 0;
    for (size_t i = 1; i < n_rows; ++i){
        const Row &r = rows[i];
        if (r.fb.rx_count == rows[i - 1].fb.rx_count) continue;
        if (prev_rx != 0) max_rx_gap_ms = std::max(max_rx_gap_ms, (r.fb.rx_ns - prev_rx) * 1e-6);
        prev_rx = r.fb.rx_ns;
        if (r.phase == CHIRP){
            if (chirp_frames == 0) chirp_first_rx = r.fb.rx_ns * 1e-9;
            chirp_last_rx = r.fb.rx_ns * 1e-9;
            chirp_frames += 1;
            max_track_err = std::max(max_track_err, (double)std::fabs(r.q_des - r.fb.q));
        }
    }
    const double chirp_fb_hz = (chirp_frames > 1 && chirp_last_rx > chirp_first_rx)
                             ? (chirp_frames - 1) / (chirp_last_rx - chirp_first_rx) : 0.0;

    // ---------- 落盘 ----------
    const bool ok_csv = write_ticks(dir, rows, n_rows, t0);
    const std::string meta_path = dir + "/meta.json";
    FILE *mf = fopen(meta_path.c_str(), "w");
    if (mf != nullptr){
        fprintf(mf, "{\n");
        fprintf(mf, "  \"schema\": \"pace_single_joint/1\",\n");
        fprintf(mf, "  \"joint_index\": %d,\n  \"motor_id\": %d,\n  \"joint_name\": \"%s\",\n", j, motor_id, json_escape(o.name).c_str());
        fprintf(mf, "  \"angle_frame\": \"motor_angle - lf1_zero_offset_rad[index]; no motor_direction/leg_offset\",\n");
        fprintf(mf, "  \"lf1_zero_offset_rad\": %.6f,\n", lf1_zero_offset_rad[j]);
        fprintf(mf, "  \"time_base\": \"CLOCK_MONOTONIC ns, relative to t0\",\n");
        fprintf(mf, "  \"t0_monotonic_ns\": %lld,\n", (long long)t0);
        fprintf(mf, "  \"kp\": %.6f,\n  \"kd\": %.6f,\n  \"qd_des\": 0.0,\n  \"tau_ff\": 0.0,\n", o.kp, o.kd);
        fprintf(mf, "  \"q0\": %.6f,\n", q0);
        fprintf(mf, "  \"chirp\": {\"type\": \"linear\", \"amp\": %.6f, \"f0\": %.6f, \"f1\": %.6f, \"duration\": %.6f, \"ramp\": %.6f},\n",
                o.amp, o.f0, o.f1, o.duration, o.ramp);
        fprintf(mf, "  \"hold_pre\": %.3f,\n  \"hold_post\": %.3f,\n  \"rate_hz\": %d,\n", o.hold_pre, o.hold_post, o.rate);
        fprintf(mf, "  \"safety\": {\"max_vel\": %.3f, \"max_tau\": %.3f, \"pos_margin\": %.3f, \"stale_ms\": %.1f},\n",
                o.max_vel, o.max_tau, o.pos_margin, o.stale_ms);
        fprintf(mf, "  \"ticks\": %zu,\n  \"overruns\": %u,\n", n_rows, overruns);
        fprintf(mf, "  \"chirp_feedback_frames\": %u,\n  \"chirp_feedback_hz\": %.2f,\n  \"max_rx_gap_ms\": %.3f,\n",
                chirp_frames, chirp_fb_hz, max_rx_gap_ms);
        fprintf(mf, "  \"completed\": %s,\n  \"abort_reason\": \"%s\"\n", abort_reason[0] == '\0' ? "true" : "false",
                json_escape(abort_reason).c_str());
        fprintf(mf, "}\n");
        fclose(mf);
    }

    printf("[PACE] %s, 共 %zu 拍, 超时 %u 拍\n", abort_reason[0] == '\0' ? "采集完成" : "已中止", n_rows, overruns);
    printf("[PACE] chirp 段反馈 %u 帧, 约 %.1f Hz, 最大反馈间隔 %.2f ms, 最大 |q_des-q| %.4f rad\n",
           chirp_frames, chirp_fb_hz, max_rx_gap_ms, max_track_err);
    printf("[PACE] 输出: %s/ticks.csv%s, meta.json%s\n", dir.c_str(), ok_csv ? "" : "(写入失败!)", mf ? "" : "(写入失败!)");

    fflush(stdout);
    close_canDevice();
    // 库里的收发线程是 detach 的, 正常 return 会在线程仍运行时析构全局对象, 用 _exit 直接退出
    _exit(abort_reason[0] == '\0' ? 0 : 3);
}

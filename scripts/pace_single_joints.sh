#!/usr/bin/env bash
# 依次对两条腿的每个关节做 PACE chirp 采集(同腿其余关节 PD 保持)。躯干吊装且无法刚性固定,
# 髋 pitch / 髋 roll 腿部惯量大, 单腿运动的反作用力会让躯干明显摆动, 因此这两个关节改为两腿镜像同时激励
# (--mirror, 每批只跑一次); 其余关节逐腿单独激励。
#
# 用法(在任意目录):
#   sudo scripts/pace_single_joints.sh                 # 右腿 -> 左腿, 每个关节运行前询问
#   scripts/pace_single_joints.sh --dry-run            # 只检查参数, 不连电机(不需要 sudo)
#   sudo scripts/pace_single_joints.sh --legs right --only hip_yaw,knee
#   sudo scripts/pace_single_joints.sh --hold body --only hip_pitch,hip_roll   # 固定头部, 全身其余关节 PD 保持
#
# 选项:
#   --legs L        right,left(默认) / right / left, 按给定顺序执行; 镜像关节只在第一条腿时运行一次
#   --only LIST     只跑这些关节: hip_pitch,hip_roll,hip_yaw,knee,ankle_pitch,ankle_roll
#   --f1 X          chirp 最高频率 [Hz], 覆盖 JOINT_TABLE 中各关节的值
#   --amp X         chirp 振幅 [rad], 覆盖 JOINT_TABLE 中各关节的值(通常配合 --only 使用)
#   --max-err X     跟踪误差中止阈值 [rad], 默认按每次的振幅取 max(0.25, 2*振幅+0.05)
#   --duration X    chirp 时长 [s], 默认 30
#   --hold SPEC     传给 pace_chirp 的 --hold: limb(默认, 吊装时同腿其余关节保持) / body(固定头部时,
#                   全身 0-28(两腿、腰、两臂)中未激励的关节全部保持, 腰 roll/pitch 为并联关节空间 PD)
#   --waist-kd-ff X 腰 roll/pitch 电机侧附加阻尼, 默认 0(只在 --hold body 时起作用)
#   --pose FILE     采集前先把参与关节移到指定位置并 PD 保持(文件为 leg_tool 角度检查输出的 "ID = n, Pos = 度" 行),
#                   采集后回到初始下垂位置再卸力; 被激励关节的中心默认取该位置(关节表中写了数值中心的除外, 如膝 0.3)
#   --pose-tol X    到达检查允许的最大误差 [rad], 默认 0.2(PD 无重力补偿, 有稳态误差)
#   --out DIR      输出根目录, 默认 build/pace_log/<日期_时间>_single
#   --gains FILE    增益文件, 默认 gains_g1_legs.txt(训练用 G1 增益)
#   --yes           不逐个询问(仍会在开始时确认一次吊装)
#   --dry-run       传给 pace_chirp 的 --dry-run
#
# JOINT_TABLE 的振幅/中心/频率按 G1 增益与 URDF 模型(scripts/urdf_dynamics.py, armature 取实测估计)设计:
#   hip_pitch  吊装时重力刚度约 19.7 Nm/rad, 自然频率约 1.24 Hz、阻尼比 0.20; 实测(2026-10-09, sym, 振幅 0.2)
#              最大跟踪误差 0.26 rad、力矩 5.8 Nm, 谐振远弱于模型预测, 振幅取 0.20、f1 = 3 Hz;
#              镜像 anti(两腿一前一后, 俯仰反力矩抵消)
#   hip_roll   自然频率约 1.87 Hz、阻尼比 0.30, 谐振处约放大 1.5 倍, f1 = 4 Hz; 镜像 sym(同时外展/内收, 左右力抵消)
#   hip_yaw / knee / ankle_pitch  自然频率约 3.2-3.7 Hz, f1 取 2-2.5 倍; 膝中心 0.3 离开伸直限位,
#              振幅 0.1 时 f1 受参考速度检查(amp*2*pi*f1 <= 0.8*max-vel)限制为 6 Hz
#   ankle_roll 自然频率约 5 Hz, f1 = 10 Hz; 踝 pitch / roll 分开跑(同时同相激励时只有一个踝电机运动)
#   踝 --max-vel 3.5: 增益表中踝最大角速度 3.9 rad/s
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(dirname "$SCRIPT_DIR")"
PACE="$ROOT/build/pace_chirp"

LEGS="right,left"
ONLY=""
F1=""
AMP=""
MAX_ERR=""
DURATION=30
HOLD="limb"
WAIST_KD_FF=0
POSE=""
POSE_TOL=""
OUT=""
GAINS="$ROOT/gains_g1_legs.txt"
ASK=1
DRY=0

# 名称  腿内偏移  振幅  中心  f1  模式(single / sym / anti)  额外参数
JOINT_TABLE=(
    "hip_pitch   0 0.20 q0  3  anti   "
    "hip_roll    1 0.05 q0  4  sym    "
    "hip_yaw     2 0.05 q0  8  single "
    "knee        3 0.10 0.3 6  single "
    "ankle_pitch 4 0.05 q0  8  single --max-vel 3.5"
    "ankle_roll  5 0.04 q0  10 single --max-vel 3.5"
)

while [ $# -gt 0 ]; do
    case "$1" in
        --legs) LEGS="$2"; shift 2 ;;
        --only) ONLY="$2"; shift 2 ;;
        --f1) F1="$2"; shift 2 ;;
        --amp) AMP="$2"; shift 2 ;;
        --max-err) MAX_ERR="$2"; shift 2 ;;
        --duration) DURATION="$2"; shift 2 ;;
        --hold) HOLD="$2"; shift 2 ;;
        --waist-kd-ff) WAIST_KD_FF="$2"; shift 2 ;;
        --pose) POSE="$2"; shift 2 ;;
        --pose-tol) POSE_TOL="$2"; shift 2 ;;
        --out) OUT="$2"; shift 2 ;;
        --gains) GAINS="$2"; shift 2 ;;
        --yes) ASK=0; shift ;;
        --dry-run) DRY=1; shift ;;
        -h|--help) sed -n '2,39p' "$0"; exit 0 ;;
        *) echo "未知参数: $1"; exit 2 ;;
    esac
done

if [ ! -x "$PACE" ]; then
    echo "找不到 $PACE, 请先在 build 目录编译"; exit 1
fi
if [ ! -f "$GAINS" ]; then
    echo "找不到增益文件 $GAINS"; exit 1
fi
POSE_ARGS=()
if [ -n "$POSE" ]; then
    [ -f "$POSE" ] || { echo "找不到 --pose 文件 $POSE"; exit 1; }
    POSE_ARGS=(--pose "$(cd "$(dirname "$POSE")" && pwd)/$(basename "$POSE")")
    [ -n "$POSE_TOL" ] && POSE_ARGS+=(--pose-tol "$POSE_TOL")
fi
if [ "$DRY" -eq 0 ] && [ "$(id -u)" -ne 0 ]; then
    echo "真机运行需要 sudo(或加 --dry-run 只检查参数)"; exit 1
fi
[ -z "$OUT" ] && OUT="$ROOT/build/pace_log/$(date +%Y%m%d_%H%M%S)_single"
mkdir -p "$OUT"
SUMMARY="$OUT/summary.txt"

if [ "$DRY" -eq 0 ]; then
    echo "即将依次对 [$LEGS] 腿采集(髋 pitch/roll 两腿镜像), f1=${F1:-按关节表} Hz, amp=${AMP:-按关节表}, duration=$DURATION s, hold=$HOLD, pose=${POSE:-无}, 输出 $OUT"
    if [ "$HOLD" = "body" ]; then
        read -r -p "确认: 头部已刚性固定、两腿与腰活动范围无障碍、急停可触达? 输入 yes 继续: " ans
    else
        read -r -p "确认: 躯干已吊装、腿部活动范围无障碍、急停可触达? 输入 yes 继续: " ans
    fi
    [ "$ans" = "yes" ] || { echo "已取消"; exit 1; }
    CONFIRM="--confirm-suspended"
else
    CONFIRM="--dry-run"
fi

printf "%-18s %-6s %-6s %-5s %-7s %-5s %s\n" "tag" "index" "amp" "f1" "mode" "exit" "run_dir" > "$SUMMARY"

run_one() {  # $1 tag, $2 index, $3 amp, $4 center, $5 f1, $6 mode, $7.. extra
    local tag="$1" idx="$2" amp="$3" center="$4" f1="$5" mode="$6"; shift 6
    local log="$OUT/$tag.log"
    local mirror=()
    [ "$mode" != "single" ] && mirror=(--mirror "$mode")
    # 高频时跟踪误差可接近 2 倍振幅, 与 pace_chirp 的参数检查一致
    local max_err="$MAX_ERR"
    [ -z "$max_err" ] && max_err=$(awk -v a="$amp" 'BEGIN { if (a < 0) a = -a; e = 2 * a + 0.05; printf "%.3f", (e > 0.25 ? e : 0.25) }')
    local cmd=("$PACE" --gains "$GAINS" --joints "$idx" --amp "$amp" --centers "$center"
               --f1 "$f1" --duration "$DURATION" --max-err "$max_err" --hold "$HOLD" --waist-kd-ff "$WAIST_KD_FF"
               "${mirror[@]}" "${POSE_ARGS[@]}" --out "$OUT" --tag "$tag" "$@" $CONFIRM)
    echo "  ${cmd[*]}"
    "${cmd[@]}" 2>&1 | tee "$log"
    local rc=${PIPESTATUS[0]}
    local dir
    dir=$(grep -o '输出: [^ ]*/ticks.csv' "$log" | sed 's/输出: //; s#/ticks.csv##')
    [ -z "$dir" ] && dir=$(grep -o '已写入 [^ ]*/plan.csv' "$log" | sed 's/已写入 //; s#/plan.csv##')
    printf "%-18s %-6s %-6s %-5s %-7s %-5s %s\n" "$tag" "$idx" "$amp" "$f1" "$mode" "$rc" "${dir:--}" >> "$SUMMARY"
    return "$rc"
}

aborted=0
IFS=',' read -r -a leg_list <<< "$LEGS"
first_leg=1
for leg in "${leg_list[@]}"; do
    case "$leg" in
        right) base=6; prefix="r" ;;
        left)  base=0; prefix="l" ;;
        *) echo "--legs 只能是 right / left: $leg"; exit 2 ;;
    esac
    for row in "${JOINT_TABLE[@]}"; do
        read -r name off amp center f1 mode extra <<< "$row"
        if [ -n "$ONLY" ] && [[ ",$ONLY," != *",$name,"* ]]; then
            continue
        fi
        [ -n "$F1" ] && f1="$F1"
        [ -n "$AMP" ] && amp="$AMP"
        idx=$((base + off))
        tag="${prefix}_${name}"
        if [ "$mode" != "single" ]; then
            # 镜像: 两腿同时运动, 每批只在第一条腿时运行一次
            [ "$first_leg" -eq 1 ] || continue
            tag="m_${name}"
        fi
        while true; do
            echo
            echo "=== [$leg] $name (下标 $idx) 振幅 $amp 中心 $center f1 $f1 Hz 模式 $mode ==="
            if [ "$ASK" -eq 1 ] && [ "$DRY" -eq 0 ]; then
                read -r -p "回车运行 / s 跳过 / q 退出: " ans
                case "$ans" in
                    s) printf "%-18s %-6s %-6s %-5s %-7s %-5s %s\n" "$tag" "$idx" "$amp" "$f1" "$mode" "skip" "-" >> "$SUMMARY"; break ;;
                    q) echo "已退出, 汇总见 $SUMMARY"; exit 0 ;;
                esac
            fi
            # shellcheck disable=SC2086
            run_one "$tag" "$idx" "$amp" "$center" "$f1" "$mode" $extra
            rc=$?
            if [ "$rc" -eq 0 ]; then
                break
            fi
            aborted=$((aborted + 1))
            echo "!!! $tag 退出码 $rc(3 为安全中止, 原因见上方输出与 meta.json)"
            if [ "$ASK" -eq 0 ] || [ "$DRY" -eq 1 ]; then
                break
            fi
            read -r -p "r 重试 / s 跳过继续 / q 退出: " ans
            case "$ans" in
                r) continue ;;
                q) echo "已退出, 汇总见 $SUMMARY"; exit 1 ;;
                *) break ;;
            esac
        done
        # 两次运行之间让腿静止, 下一次 probe 要求关节不动(--hold body 时手臂和悬垂的下半身停得慢, 多等一会)
        if [ "$DRY" -eq 0 ]; then
            if [ "$HOLD" = "body" ]; then sleep 6; else sleep 3; fi
        fi
    done
    first_leg=0
done

echo
echo "==== 汇总 ($SUMMARY) ===="
cat "$SUMMARY"
[ "$aborted" -gt 0 ] && echo "有 $aborted 次运行未正常完成"
exit 0

#!/usr/bin/env bash
# 依次对两条腿的每个关节单独做 PACE chirp 采集(同腿其余关节 PD 保持), 用于躯干吊装、多关节同时运动晃动太大的情况。
#
# 用法(在任意目录):
#   sudo scripts/pace_single_joints.sh                 # 右腿 -> 左腿, 每个关节运行前询问
#   scripts/pace_single_joints.sh --dry-run            # 只检查参数, 不连电机(不需要 sudo)
#   sudo scripts/pace_single_joints.sh --legs right --only hip_pitch,knee --f1 2
#
# 选项:
#   --legs L        right,left(默认) / right / left, 按给定顺序执行
#   --only LIST     只跑这些关节: hip_pitch,hip_roll,hip_yaw,knee,ankle_pitch,ankle_roll
#   --f1 X          chirp 最高频率 [Hz], 默认 3
#   --duration X    chirp 时长 [s], 默认 30
#   --out DIR       输出根目录, 默认 build/pace_log/<日期_时间>_single
#   --gains FILE    增益文件, 默认 gains_yaoguwu_plus_122500i.txt
#   --yes           不逐个询问(仍会在开始时确认一次吊装)
#   --dry-run       传给 pace_chirp 的 --dry-run
#
# 每个关节的振幅/中心见下方 JOINT_TABLE, 依据:
#   hip_pitch 0.12: 实测膝伸直时髋 pitch 在 q0 约 ±0.10 rad 处有挡点, 低频实际运动约为指令的 0.6 倍
#   knee 中心 0.3: 离开伸直限位(假设两腿膝关节 URDF 正方向均为弯曲, 首次运行左膝时注意观察 move_in 方向)
#   ankle pitch / roll 分开跑: 两者同时同相激励时只有一个踝电机运动
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(dirname "$SCRIPT_DIR")"
PACE="$ROOT/build/pace_chirp"

LEGS="right,left"
ONLY=""
F1=3
DURATION=30
OUT=""
GAINS="$ROOT/gains_yaoguwu_plus_122500i.txt"
ASK=1
DRY=0

# 名称  腿内偏移  振幅  中心  额外参数
JOINT_TABLE=(
    "hip_pitch   0 0.12 q0  --max-err 0.3"
    "hip_roll    1 0.05 q0  "
    "hip_yaw     2 0.05 q0  "
    "knee        3 0.10 0.3 "
    "ankle_pitch 4 0.06 q0  "
    "ankle_roll  5 0.05 q0  "
)

while [ $# -gt 0 ]; do
    case "$1" in
        --legs) LEGS="$2"; shift 2 ;;
        --only) ONLY="$2"; shift 2 ;;
        --f1) F1="$2"; shift 2 ;;
        --duration) DURATION="$2"; shift 2 ;;
        --out) OUT="$2"; shift 2 ;;
        --gains) GAINS="$2"; shift 2 ;;
        --yes) ASK=0; shift ;;
        --dry-run) DRY=1; shift ;;
        -h|--help) sed -n '2,24p' "$0"; exit 0 ;;
        *) echo "未知参数: $1"; exit 2 ;;
    esac
done

if [ ! -x "$PACE" ]; then
    echo "找不到 $PACE, 请先在 build 目录编译"; exit 1
fi
if [ ! -f "$GAINS" ]; then
    echo "找不到增益文件 $GAINS"; exit 1
fi
if [ "$DRY" -eq 0 ] && [ "$(id -u)" -ne 0 ]; then
    echo "真机运行需要 sudo(或加 --dry-run 只检查参数)"; exit 1
fi
[ -z "$OUT" ] && OUT="$ROOT/build/pace_log/$(date +%Y%m%d_%H%M%S)_single"
mkdir -p "$OUT"
SUMMARY="$OUT/summary.txt"

if [ "$DRY" -eq 0 ]; then
    echo "即将依次对 [$LEGS] 腿逐关节采集, f1=$F1 Hz, duration=$DURATION s, 输出 $OUT"
    read -r -p "确认: 躯干已吊装、腿部活动范围无障碍、急停可触达? 输入 yes 继续: " ans
    [ "$ans" = "yes" ] || { echo "已取消"; exit 1; }
    CONFIRM="--confirm-suspended"
else
    CONFIRM="--dry-run"
fi

printf "%-18s %-6s %-6s %-5s %s\n" "tag" "index" "amp" "exit" "run_dir" > "$SUMMARY"

run_one() {  # $1 tag, $2 index, $3 amp, $4 center, $5.. extra
    local tag="$1" idx="$2" amp="$3" center="$4"; shift 4
    local log="$OUT/$tag.log"
    local cmd=("$PACE" --gains "$GAINS" --joints "$idx" --amp "$amp" --centers "$center"
               --f1 "$F1" --duration "$DURATION" --out "$OUT" --tag "$tag" "$@" $CONFIRM)
    echo "  ${cmd[*]}"
    "${cmd[@]}" 2>&1 | tee "$log"
    local rc=${PIPESTATUS[0]}
    local dir
    dir=$(grep -o '输出: [^ ]*/ticks.csv' "$log" | sed 's/输出: //; s#/ticks.csv##')
    [ -z "$dir" ] && dir=$(grep -o '已写入 [^ ]*/plan.csv' "$log" | sed 's/已写入 //; s#/plan.csv##')
    printf "%-18s %-6s %-6s %-5s %s\n" "$tag" "$idx" "$amp" "$rc" "${dir:--}" >> "$SUMMARY"
    return "$rc"
}

aborted=0
IFS=',' read -r -a leg_list <<< "$LEGS"
for leg in "${leg_list[@]}"; do
    case "$leg" in
        right) base=6; prefix="r" ;;
        left)  base=0; prefix="l" ;;
        *) echo "--legs 只能是 right / left: $leg"; exit 2 ;;
    esac
    for row in "${JOINT_TABLE[@]}"; do
        read -r name off amp center extra <<< "$row"
        if [ -n "$ONLY" ] && [[ ",$ONLY," != *",$name,"* ]]; then
            continue
        fi
        idx=$((base + off))
        tag="${prefix}_${name}"
        while true; do
            echo
            echo "=== [$leg] $name (下标 $idx) 振幅 $amp 中心 $center ==="
            if [ "$ASK" -eq 1 ] && [ "$DRY" -eq 0 ]; then
                read -r -p "回车运行 / s 跳过 / q 退出: " ans
                case "$ans" in
                    s) printf "%-18s %-6s %-6s %-5s %s\n" "$tag" "$idx" "$amp" "skip" "-" >> "$SUMMARY"; break ;;
                    q) echo "已退出, 汇总见 $SUMMARY"; exit 0 ;;
                esac
            fi
            # shellcheck disable=SC2086
            run_one "$tag" "$idx" "$amp" "$center" $extra
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
        # 两次运行之间让腿静止, 下一次 probe 要求关节不动
        [ "$DRY" -eq 0 ] && sleep 3
    done
done

echo
echo "==== 汇总 ($SUMMARY) ===="
cat "$SUMMARY"
[ "$aborted" -gt 0 ] && echo "有 $aborted 次运行未正常完成"
exit 0

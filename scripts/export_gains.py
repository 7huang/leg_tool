#!/usr/bin/env python3
"""
从策略 ONNX 的 metadata 导出 29 个关节的 kp/kd, 按 MotorCmd 下标(即 deploy_real 中 joint_xml)顺序,
供 pace_chirp --gains 使用。

用法:
    python3 export_gains.py <policy.onnx> [-o gains.txt]

metadata 中 joint_names / joint_stiffness / joint_damping 为策略关节顺序, 这里按关节名映射到下标顺序。
deploy_real/task/*.py 中的 leg_kp_scale 等缩放系数当前均为 1.0, 这里不做缩放。
"""
import argparse
import sys

# 与 deploy_real/task/*.py 的 joint_xml 一致
JOINT_XML = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint", "left_knee_joint",
    "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint", "right_knee_joint",
    "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint",
]


def read_varint(buf, pos):
    result, shift = 0, 0
    while True:
        b = buf[pos]
        pos += 1
        result |= (b & 0x7F) << shift
        if not b & 0x80:
            return result, pos
        shift += 7


def parse_fields(buf):
    """逐个产出 protobuf 消息的 (字段号, 线类型, 值); 长度定界字段的值为 bytes"""
    pos = 0
    while pos < len(buf):
        key, pos = read_varint(buf, pos)
        field, wire = key >> 3, key & 7
        if wire == 0:
            value, pos = read_varint(buf, pos)
        elif wire == 1:
            value, pos = buf[pos:pos + 8], pos + 8
        elif wire == 2:
            length, pos = read_varint(buf, pos)
            value, pos = buf[pos:pos + length], pos + length
        elif wire == 5:
            value, pos = buf[pos:pos + 4], pos + 4
        else:
            raise ValueError(f"不支持的 protobuf 线类型 {wire}")
        yield field, wire, value


def read_metadata_raw(path):
    """不依赖 onnx 库: ModelProto 的 metadata_props 是字段 14, 每项 StringStringEntryProto 的 key=1, value=2"""
    with open(path, "rb") as f:
        buf = f.read()
    meta = {}
    for field, wire, value in parse_fields(buf):
        if field == 14 and wire == 2:
            entry = {k: v.decode("utf-8") for k, w, v in parse_fields(value) if w == 2}
            meta[entry.get(1, "")] = entry.get(2, "")
    return meta


def read_metadata(path):
    try:
        import onnxruntime
        return onnxruntime.InferenceSession(path).get_modelmeta().custom_metadata_map
    except ImportError:
        pass
    try:
        import onnx
        return {p.key: p.value for p in onnx.load(path).metadata_props}
    except ImportError:
        return read_metadata_raw(path)


def main():
    parser = argparse.ArgumentParser(description="从策略 ONNX 导出 kp/kd")
    parser.add_argument("onnx", help="策略 ONNX 文件")
    parser.add_argument("-o", "--output", help="输出文件, 默认打印到终端")
    args = parser.parse_args()

    meta = read_metadata(args.onnx)
    for key in ("joint_names", "joint_stiffness", "joint_damping"):
        if key not in meta:
            sys.exit(f"ONNX metadata 缺少 {key}")
    names = meta["joint_names"].split(",")
    kp = [float(x) for x in meta["joint_stiffness"].split(",")]
    kd = [float(x) for x in meta["joint_damping"].split(",")]
    if not (len(names) == len(kp) == len(kd)):
        sys.exit("joint_names / joint_stiffness / joint_damping 长度不一致")
    missing = [n for n in JOINT_XML if n not in names]
    if missing:
        # 策略不含的关节写成注释; pace_chirp 只有在用到这些关节时才会报缺少增益
        print(f"注意: 策略中没有这些关节, 已写成注释: {', '.join(missing)}", file=sys.stderr)

    lines = [f"# source: {args.onnx}", "# index name kp kd"]
    for i, name in enumerate(JOINT_XML):
        if name in missing:
            lines.append(f"# {i} {name} (策略中无此关节)")
            continue
        j = names.index(name)
        lines.append(f"{i} {name} {kp[j]:g} {kd[j]:g}")
    text = "\n".join(lines) + "\n"
    if args.output:
        with open(args.output, "w", encoding="utf-8") as f:
            f.write(text)
        print(f"已写入 {args.output}")
    else:
        print(text, end="")


if __name__ == "__main__":
    main()

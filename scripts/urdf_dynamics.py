#!/usr/bin/env python3
"""
从 URDF 计算吊装(基座固定、躯干竖直)姿态下各关节的等效连杆惯量与重力刚度, 供 PACE 实验设计与结果检查。

对关节 j, 把其后代全部连杆视为一个刚体(其余关节锁定), 计算:
    I_link : 绕关节轴的转动惯量 [kg*m^2](不含电机转子折算惯量 armature, 后者由 PACE 辨识)
    k_g    : 重力刚度 -d(tau_g)/dq [Nm/rad], 正值表示重力有回复作用(像单摆)
    tau_g  : 该姿态下的重力力矩 [Nm]

用法:
    python3 urdf_dynamics.py <robot.urdf> [--pose meta.json] [--joints 名称,...]
--pose 可传 pace_chirp 的 meta.json, 用其中各关节 q0 作为姿态(其余关节为 0)。
"""
import argparse
import json
import xml.etree.ElementTree as ET

import numpy as np

GRAVITY = np.array([0.0, 0.0, -9.81])


def rpy_to_matrix(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def axis_angle(axis, q):
    a = axis / np.linalg.norm(axis)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(q) * K + (1 - np.cos(q)) * K @ K


def vec(s, default="0 0 0"):
    return np.array([float(x) for x in (s or default).split()])


class Robot:
    def __init__(self, path):
        root = ET.parse(path).getroot()
        self.links = {}
        for link in root.findall("link"):
            inertial = link.find("inertial")
            if inertial is None:
                self.links[link.get("name")] = None
                continue
            o = inertial.find("origin")
            i = inertial.find("inertia").attrib
            Ic = np.array([[float(i["ixx"]), float(i["ixy"]), float(i["ixz"])],
                           [float(i["ixy"]), float(i["iyy"]), float(i["iyz"])],
                           [float(i["ixz"]), float(i["iyz"]), float(i["izz"])]])
            R = rpy_to_matrix(vec(o.get("rpy") if o is not None else None))
            self.links[link.get("name")] = {
                "mass": float(inertial.find("mass").get("value")),
                "com": vec(o.get("xyz") if o is not None else None),
                "inertia": R @ Ic @ R.T,  # 转到连杆坐标系
            }
        self.joints = {}
        self.children = {}
        for j in root.findall("joint"):
            o = j.find("origin")
            ax = j.find("axis")
            lim = j.find("limit")
            info = {
                "name": j.get("name"),
                "type": j.get("type"),
                "parent": j.find("parent").get("link"),
                "child": j.find("child").get("link"),
                "xyz": vec(o.get("xyz") if o is not None else None),
                "R": rpy_to_matrix(vec(o.get("rpy") if o is not None else None)),
                "axis": vec(ax.get("xyz") if ax is not None else None, "1 0 0"),
                "lower": float(lim.get("lower")) if lim is not None and lim.get("lower") else None,
                "upper": float(lim.get("upper")) if lim is not None and lim.get("upper") else None,
                "effort": float(lim.get("effort")) if lim is not None and lim.get("effort") else None,
                "velocity": float(lim.get("velocity")) if lim is not None and lim.get("velocity") else None,
            }
            self.joints[info["name"]] = info
            self.children.setdefault(info["parent"], []).append(info)
        child_links = {j["child"] for j in self.joints.values()}
        self.root_link = next(name for name in self.links if name not in child_links)

    def forward(self, q):
        """返回 {连杆: (R, p)} 与 {关节: (轴方向, 轴上一点)}, 均为世界坐标(根连杆为原点、姿态单位阵)"""
        frames = {self.root_link: (np.eye(3), np.zeros(3))}
        joint_frames = {}
        stack = [self.root_link]
        while stack:
            parent = stack.pop()
            Rp, pp = frames[parent]
            for j in self.children.get(parent, []):
                Rj = Rp @ j["R"]
                pj = pp + Rp @ j["xyz"]
                axis_w = Rj @ (j["axis"] / np.linalg.norm(j["axis"]))
                joint_frames[j["name"]] = (axis_w, pj)
                Rc = Rj @ axis_angle(j["axis"], q.get(j["name"], 0.0)) if j["type"] != "fixed" else Rj
                frames[j["child"]] = (Rc, pj)
                stack.append(j["child"])
        return frames, joint_frames

    def subtree_links(self, joint_name):
        out, stack = [], [self.joints[joint_name]["child"]]
        while stack:
            link = stack.pop()
            out.append(link)
            stack.extend(j["child"] for j in self.children.get(link, []))
        return out

    def joint_terms(self, joint_name, q):
        frames, joint_frames = self.forward(q)
        a, p0 = joint_frames[joint_name]
        I_axis = tau = dtau = mass = 0.0
        for link in self.subtree_links(joint_name):
            info = self.links.get(link)
            if info is None:
                continue
            R, p = frames[link]
            m = info["mass"]
            r = p + R @ info["com"] - p0
            r_perp = r - (a @ r) * a
            I_axis += a @ (R @ info["inertia"] @ R.T) @ a + m * (r_perp @ r_perp)
            tau += a @ np.cross(r, m * GRAVITY)
            dtau += a @ np.cross(np.cross(a, r), m * GRAVITY)
            mass += m
        return {"I_link": I_axis, "k_g": -dtau, "tau_g": tau, "mass": mass}


def main():
    parser = argparse.ArgumentParser(description="URDF 吊装姿态下的关节等效惯量与重力刚度")
    parser.add_argument("urdf")
    parser.add_argument("--pose", help="pace_chirp 的 meta.json, 使用其中各关节 q0")
    parser.add_argument("--joints", help="逗号分隔的关节名, 默认全部腿部关节")
    args = parser.parse_args()

    robot = Robot(args.urdf)
    q = {}
    if args.pose:
        meta = json.load(open(args.pose, encoding="utf-8"))
        q = {j["name"]: j["q0"] for j in meta["joints"]}
    names = args.joints.split(",") if args.joints else [n for n in robot.joints if "hip" in n or "knee" in n or "ankle" in n]
    print(f"{'joint':26}{'q':>8}{'子树质量':>9}{'I_link':>9}{'k_g':>8}{'tau_g':>8}{'限位':>16}")
    for n in names:
        t = robot.joint_terms(n, q)
        j = robot.joints[n]
        print(f"{n:26}{q.get(n, 0.0):8.3f}{t['mass']:9.3f}{t['I_link']:9.4f}{t['k_g']:8.2f}{t['tau_g']:8.2f}"
              f"{'[%.2f, %.2f]' % (j['lower'], j['upper']):>16}")


if __name__ == "__main__":
    main()

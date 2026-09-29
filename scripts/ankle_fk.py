"""
踝(并联)正解的 numpy 实现, 与 src/parallelmechanism.cpp 的 fk/jac 使用相同的几何参数与电机零位偏置,
但迭代到严格收敛。

底层 fk 以上一帧解为初值, 杆长误差平方 < 1e-8(约 0.1 mm)即停止; 电机转一个编码器步(3.8e-4 rad)
杆长只变约 0.009 mm, 因此解算出的关节角呈台阶状(实测误差 RMS 约 1 mrad, 最大约 3 mrad)。
这里用记录的电机原始角(ticks.csv 的 motor_q_<m> 列)重新解算。
"""
import numpy as np

# 列顺序: side 1 左踝, 2 右踝, 3 腰
MOTOR_CONTER_1 = np.array([[-0.04475, -0.04475, -0.0435],
                           [0.00000, 0.00000, -0.0375],
                           [0.23885, 0.23885, 0.0480]])
MOTOR_CONTER_2 = np.array([[-0.04475, -0.04475, -0.0435],
                           [0.00000, 0.00000, 0.0375],
                           [0.17785, 0.17785, 0.0480]])
S_DOWN_1 = np.array([[-0.03175, -0.03175, -0.0435],
                     [0.021735, -0.021735, -0.0280],
                     [-0.00100, -0.00100, -0.0070]])
S_DOWN_2 = np.array([[-0.03175, -0.03175, -0.0435],
                     [-0.021735, 0.021735, 0.0280],
                     [-0.00100, -0.00100, -0.0070]])
CRANK = np.array([0.0225, 0.0225, 0.0245])
ROD_1 = np.array([0.24601, 0.24601, 0.055])
ROD_2 = np.array([0.1851, 0.1851, 0.055])

# 下标: 踝 pitch 关节 -> (side, pitch 电机下标, roll 电机下标, roll 关节下标)
ANKLES = {4: (1, 4, 5, 5), 10: (2, 10, 11, 11)}


def _rod(pitch, roll, motor, col, conter, s_down):
    """杆向量 r 及其对 (pitch, roll) 的偏导, 与 C++ jac 中 J_l2pr 的一行一致"""
    cp, sp, cr, sr = np.cos(pitch), np.sin(pitch), np.cos(roll), np.sin(roll)
    x, y, z = s_down[:, col]
    gx = cp * x + sp * sr * y + sp * cr * z
    gy = cr * y - sr * z
    gz = -sp * x + cp * sr * y + cp * cr * z
    crk = CRANK[col]
    rx = gx - conter[0, col]
    ry = gy - conter[1, col] - crk * np.cos(motor)
    rz = gz - conter[2, col] - crk * np.sin(motor)
    L = np.sqrt(rx * rx + ry * ry + rz * rz)
    ex, ey, ez = rx / L, ry / L, rz / L
    A = sr * y + cr * z
    B = cr * y - sr * z
    dp = ex * (cp * A - sp * x) + ez * (-sp * A - cp * x)
    dr = ex * (sp * B) - ey * A + ez * (cp * B)
    return L, dp, dr


def motor_to_joint(side, motor_a, motor_b, pitch0, roll0, iters=20, tol=1e-12):
    """
    motor_a / motor_b: pitch 下标电机与 roll 下标电机的角度(get_motor_data 坐标, 即 motor_q_<m> 列)
    pitch0 / roll0: 迭代初值(例如底层解算结果 q_<i>), 返回严格收敛的 (pitch, roll) [rad]
    """
    col = side - 1
    # 与 convert_motor_data + motorToJointLeft/Right 相同: 电机角取反后加零位偏置
    if side == 1:
        t1 = -motor_a + 0.261
        t2 = -motor_b + np.pi - 0.261
    else:
        t1 = -motor_a + np.pi - 0.261
        t2 = -motor_b + 0.261
    p = np.array(pitch0, dtype=float)
    r = np.array(roll0, dtype=float)
    for _ in range(iters):
        L1, a, b = _rod(p, r, t1, col, MOTOR_CONTER_1, S_DOWN_1)
        L2, c, d = _rod(p, r, t2, col, MOTOR_CONTER_2, S_DOWN_2)
        e1 = L1 - ROD_1[col]
        e2 = L2 - ROD_2[col]
        det = a * d - b * c
        dp = (d * -e1 - b * -e2) / det
        dr = (-c * -e1 + a * -e2) / det
        p = p + dp
        r = r + dr
        if np.nanmax(np.abs(dp) + np.abs(dr)) < tol:
            break
    return p, r

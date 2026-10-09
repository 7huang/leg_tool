#include "parallelmechanism.h"
#include <algorithm>
#include <iostream>

namespace {

// 2x2 线性方程组 [a b; c d] * [x; y] = [bx; by] 的解析解，
// 避免为 2x2 系统引入 colPivHouseholderQr 的开销。
inline bool solve2x2(double a, double b, double c, double d,
                     double bx, double by, double& x, double& y,
                     double eps = 1e-12)
{
    const double det = a * d - b * c;
    if (std::abs(det) < eps)
    {
        x = 0.0;
        y = 0.0;
        return false;
    }
    const double inv_det = 1.0 / det;
    x = inv_det * (d * bx - b * by);
    y = inv_det * (-c * bx + a * by);
    return true;
}

} // namespace

ParallelMechanism::ParallelMechanism()
{
    // << 只能写在函数体内部！
    motor_conter_1 <<
        -0.04475, -0.04475, -0.0435,
         0.00000,  0.00000, -0.0375,
         0.23885,  0.23885,  0.0480;

    motor_conter_2 <<
        -0.04475, -0.04475, -0.0435,
         0.00000,  0.00000,  0.0375,
         0.17785,  0.17785,  0.0480;

    S_down_1 <<
        -0.03175, -0.03175, -0.0435,
         0.021735, -0.021735, -0.0280,
        -0.00100, -0.00100, -0.0070;

    S_down_2 <<
        -0.03175, -0.03175, -0.0435,
        -0.021735, 0.021735, 0.0280,
        -0.00100, -0.00100, -0.0070;
}



Matrix3d ParallelMechanism::Ry(double theta) const
{
    double c = cos(theta);
    double s = sin(theta);
    Matrix3d R;
    R << c, 0, s,
         0, 1, 0,
        -s, 0, c;
    return R;
}

Matrix3d ParallelMechanism::Rx(double theta) const
{
    double c = cos(theta);
    double s = sin(theta);
    Matrix3d R;
    R << 1, 0, 0,
         0, c, -s,
         0, s, c;
    return R;
}

Matrix3d ParallelMechanism::Ryx(double pitch, double roll) const
{
    return Ry(pitch) * Rx(roll);
}

Matrix3d ParallelMechanism::VectoT(const Vector3d& vec) const
{
    Matrix3d skew;
    skew << 0,      -vec(2),  vec(1),
            vec(2),  0,      -vec(0),
           -vec(1),  vec(0),  0;
    return skew;
}

int ParallelMechanism::sincos(double A, double B, double C, double& theta, double& fai) const
{
    double R = std::sqrt(A*A + B*B);
    const double eps = 1e-10;
    if (R < eps)
    {
        theta = NAN;
        fai = NAN;
        return 1;
    }
    double ratio = C / R;
    if (std::fabs(ratio) > 1.0001)
    {
        return 1;
    }
    fai = std::atan2(B, A);
    double val_clamp = std::max(std::min(ratio, 1.0), -1.0);
    theta = std::asin(val_clamp);
    return 0;
}

int ParallelMechanism::ik(double pitch_angle, double roll_angle,
                          double motor_1_angle_bef, double motor_2_angle_bef,
                          int side, double& motor_1_angle, double& motor_2_angle)
{
    int error_state = 0;

    // Ryx = Ry(pitch) * Rx(roll):
    //   [cp,  sp*sr,  sp*cr]
    //   [ 0,     cr,    -sr]
    //   [-sp,  cp*sr,  cp*cr]
    const double cp = std::cos(pitch_angle);
    const double sp = std::sin(pitch_angle);
    const double cr = std::cos(roll_angle);
    const double sr = std::sin(roll_angle);

    const int col = side - 1;
    const double x1 = S_down_1(0, col), y1 = S_down_1(1, col), z1 = S_down_1(2, col);
    const double x2 = S_down_2(0, col), y2 = S_down_2(1, col), z2 = S_down_2(2, col);

    // Sd_g = Ryx * S（手写展开）
    double d1x = cp*x1 + sp*sr*y1 + sp*cr*z1;
    double d1y = cr*y1 - sr*z1;
    double d1z = -sp*x1 + cp*sr*y1 + cp*cr*z1;
    double d2x = cp*x2 + sp*sr*y2 + sp*cr*z2;
    double d2y = cr*y2 - sr*z2;
    double d2z = -sp*x2 + cp*sr*y2 + cp*cr*z2;

    double del1x = motor_conter_1(0, col) - d1x;
    double del1y = motor_conter_1(1, col) - d1y;
    double del1z = motor_conter_1(2, col) - d1z;
    double del2x = motor_conter_2(0, col) - d2x;
    double del2y = motor_conter_2(1, col) - d2y;
    double del2z = motor_conter_2(2, col) - d2z;

    double A1 = del1z;
    double B1 = del1y;
    double C1 = (rod_1(col)*rod_1(col) - (del1x*del1x + del1y*del1y + del1z*del1z)
                 - crank(col)*crank(col)) / (2.0 * crank(col));

    double A2 = del2z;
    double B2 = del2y;
    double C2 = (rod_2(col)*rod_2(col) - (del2x*del2x + del2y*del2y + del2z*del2z)
                 - crank(col)*crank(col)) / (2.0 * crank(col));

    double th1, fa1, th2, fa2;
    int err1 = sincos(A1, B1, C1, th1, fa1);
    int err2 = sincos(A2, B2, C2, th2, fa2);

    if (err1 || err2)
    {
        error_state = 1;
        motor_1_angle = motor_1_angle_bef;
        motor_2_angle = motor_2_angle_bef;
    }
    else
    {
        if (side == 1 || side == 3)
        {
            motor_1_angle = th1 - fa1;
            motor_2_angle = M_PI - th2 - fa2;
        }
        else if (side == 2)
        {
            motor_1_angle = M_PI - th1 - fa1;
            motor_2_angle = th2 - fa2;
        }
    }
    return error_state;
}

void ParallelMechanism::jac(double motor_1_angle, double motor_2_angle,
                            double pitch_angle, double roll_angle, int side,
                            Eigen::Matrix2d& J, Eigen::Matrix2d& J_l2pr,
                            Eigen::Vector2d& L, int& flag)
{
    flag = 0;

    // Ryx = Ry(pitch) * Rx(roll):
    //   [cp,  sp*sr,  sp*cr]
    //   [ 0,     cr,    -sr]
    //   [-sp,  cp*sr,  cp*cr]
    const double cp = std::cos(pitch_angle);
    const double sp = std::sin(pitch_angle);
    const double cr = std::cos(roll_angle);
    const double sr = std::sin(roll_angle);

    const int col = side - 1;
    const double x1 = S_down_1(0, col), y1 = S_down_1(1, col), z1 = S_down_1(2, col);
    const double x2 = S_down_2(0, col), y2 = S_down_2(1, col), z2 = S_down_2(2, col);

    // Sd_g = Ryx * S（手写展开）
    double gx1 = cp*x1 + sp*sr*y1 + sp*cr*z1;
    double gy1 = cr*y1 - sr*z1;
    double gz1 = -sp*x1 + cp*sr*y1 + cp*cr*z1;
    double gx2 = cp*x2 + sp*sr*y2 + sp*cr*z2;
    double gy2 = cr*y2 - sr*z2;
    double gz2 = -sp*x2 + cp*sr*y2 + cp*cr*z2;

    const double crk = crank(col);
    const double c1 = std::cos(motor_1_angle), s1 = std::sin(motor_1_angle);
    const double c2 = std::cos(motor_2_angle), s2 = std::sin(motor_2_angle);

    const double mx1 = motor_conter_1(0, col), my1 = motor_conter_1(1, col), mz1 = motor_conter_1(2, col);
    const double mx2 = motor_conter_2(0, col), my2 = motor_conter_2(1, col), mz2 = motor_conter_2(2, col);

    // r_rod = Sd_g - Su_g，Su_g = motor_conter + r_crank
    double rx1 = gx1 - mx1;
    double ry1 = gy1 - my1 - crk * c1;
    double rz1 = gz1 - mz1 - crk * s1;
    double rx2 = gx2 - mx2;
    double ry2 = gy2 - my2 - crk * c2;
    double rz2 = gz2 - mz2 - crk * s2;

    double L1 = std::sqrt(rx1*rx1 + ry1*ry1 + rz1*rz1);
    double L2 = std::sqrt(rx2*rx2 + ry2*ry2 + rz2*rz2);
    L(0) = L1;
    L(1) = L2;

    double e1x = rx1 / L1, e1y = ry1 / L1, e1z = rz1 / L1;
    double e2x = rx2 / L2, e2y = ry2 / L2, e2z = rz2 / L2;

    // col1 = VectoT(y) * Rx_roll * S = (A, 0, -x)
    // col2 = VectoT(s) * Rx_roll * S = (0, -A, B)，其中 A = sr*y + cr*z, B = cr*y - sr*z
    const double A1 = sr*y1 + cr*z1;
    const double B1 = cr*y1 - sr*z1;
    const double A2 = sr*y2 + cr*z2;
    const double B2 = cr*y2 - sr*z2;

    // part.row = ep * Ry(pitch) * [col1 | col2]
    // Ry(pitch)*col1 = (cp*A - sp*x, 0, -sp*A - cp*x)
    // Ry(pitch)*col2 = (sp*B, -A, cp*B)
    const double p00 = e1x * (cp*A1 - sp*x1) + e1z * (-sp*A1 - cp*x1);
    const double p01 = e1x * (sp*B1) - e1y * A1 + e1z * (cp*B1);
    const double p10 = e2x * (cp*A2 - sp*x2) + e2z * (-sp*A2 - cp*x2);
    const double p11 = e2x * (sp*B2) - e2y * A2 + e2z * (cp*B2);

    J_l2pr(0,0) = p00;
    J_l2pr(0,1) = p01;
    J_l2pr(1,0) = p10;
    J_l2pr(1,1) = p11;

    // den = ep * (VectoT(s) * r_crank)，VectoT(s)*r_crank = (0, -crk*s, crk*c)
    double den1 = crk * (-e1y * s1 + e1z * c1);
    double den2 = crk * (-e2y * s2 + e2z * c2);
    const double eps = 1e-12;

    if (std::abs(den1) < eps || std::abs(den2) < eps)
    {
        flag = 1;
        J.setIdentity();
        return;
    }

    const double inv_den1 = 1.0 / den1;
    const double inv_den2 = 1.0 / den2;
    J(0,0) = p00 * inv_den1;
    J(0,1) = p01 * inv_den1;
    J(1,0) = p10 * inv_den2;
    J(1,1) = p11 * inv_den2;
}

void ParallelMechanism::fk(double motor_1_angle, double motor_2_angle,
                           double pitch_angle_bef, double roll_angle_bef, int side,
                           double& pitch_angle, double& roll_angle)
{
    const double error_ref = 1e-8;
    const int step_max = 30;
    const double lam = 1e-3;
    Eigen::Vector2d pr;
    pr << pitch_angle_bef, roll_angle_bef;
    Eigen::Vector2d L_init;
    L_init << rod_1(side - 1), rod_2(side - 1);

    for (int i = 0; i < step_max; ++i)
    {
        Eigen::Matrix2d J_l2pr, J;
        Eigen::Vector2d L;
        int flag;
        jac(motor_1_angle, motor_2_angle, pr(0), pr(1), side, J, J_l2pr, L, flag);
        Eigen::Vector2d err = L - L_init;
        double cost = err.squaredNorm();
        if (cost < error_ref) break;

        Eigen::Vector2d delta;
        if (flag == 0)
        {
            // delta = J_l2pr^{-1} * (-err)，2x2 解析逆
            double dx, dy;
            solve2x2(J_l2pr(0,0), J_l2pr(0,1), J_l2pr(1,0), J_l2pr(1,1),
                     -err(0), -err(1), dx, dy);
            delta << dx, dy;
        }
        else
        {
            // 阻尼最小二乘：A = J_l2pr^T J_l2pr + lam*I，b = -J_l2pr^T * err，2x2 解析解
            const double a00 = J_l2pr(0,0), a01 = J_l2pr(0,1);
            const double a10 = J_l2pr(1,0), a11 = J_l2pr(1,1);
            const double A00 = a00*a00 + a10*a10 + lam;
            const double A01 = a00*a01 + a10*a11;
            const double A11 = a01*a01 + a11*a11 + lam;
            const double b0 = -(a00*err(0) + a10*err(1));
            const double b1 = -(a01*err(0) + a11*err(1));
            double dx, dy;
            solve2x2(A00, A01, A01, A11, b0, b1, dx, dy);
            delta << dx, dy;
        }
        pr += delta;
        if (delta.norm() < error_ref) break;
    }
    pitch_angle = pr(0);
    roll_angle = pr(1);
}

void ParallelMechanism::velocity_pr2motor(const Eigen::Matrix2d& J,
                                          double pitch_vel, double roll_vel,
                                          double& m1_vel, double& m2_vel)
{
    Eigen::Vector2d v;
    v = J * Eigen::Vector2d(pitch_vel, roll_vel);
    m1_vel = v(0);
    m2_vel = v(1);
}

void ParallelMechanism::velocity_motor2pr(const Eigen::Matrix2d& J,
                                          double m1_vel, double m2_vel,
                                          double& pitch_vel, double& roll_vel)
{
    // res = J^{-1} * v，2x2 解析逆
    double pv, rv;
    solve2x2(J(0,0), J(0,1), J(1,0), J(1,1), m1_vel, m2_vel, pv, rv);
    pitch_vel = pv;
    roll_vel = rv;
}

void ParallelMechanism::torque_pr2motor(const Eigen::Matrix2d& J,
                                         double pitch_tau, double roll_tau,
                                         double& m1_tau, double& m2_tau)
{
    // tau_motor = (J^T)^{-1} * tau，等价于求解 J^T * z = tau，2x2 解析逆
    double t1, t2;
    solve2x2(J(0,0), J(1,0), J(0,1), J(1,1), pitch_tau, roll_tau, t1, t2);
    m1_tau = t1;
    m2_tau = t2;
}

void ParallelMechanism::torque_motor2pr(const Eigen::Matrix2d& J,
                                        double m1_tau, double m2_tau,
                                        double& pitch_tau, double& roll_tau)
{
    Eigen::Vector2d tau_task = J.transpose() * Eigen::Vector2d(m1_tau, m2_tau);
    pitch_tau = tau_task(0);
    roll_tau = tau_task(1);
}

double last_motor_4 = 0.0;
double last_motor_5 = 0.0;
double last_motor_10 = 0.0;
double last_motor_11 = 0.0;
double last_motor_13 = 0.0;
double last_motor_14 = 0.0;

std::tuple<double, double, double, double, double, double> 
    ParallelMechanism::jointToMotorLeft(double tr, double tp, double vr, double vp,
                     double tq_r, double tq_p){
        int side = 1;
        double motor_4_angle = 0.0;
        double motor_5_angle = 0.0;
        double motor_4_tff = 0.0;
        double motor_5_tff = 0.0;
        Eigen::Matrix2d J;
        Eigen::Matrix2d J_l2pr;
        Eigen::Vector2d L;
        int flag = 0;
        ik(tp,tr,last_motor_4,last_motor_5,side,motor_4_angle,motor_5_angle);
        last_motor_4 = motor_4_angle;
        last_motor_5 = motor_5_angle;
        jac(motor_4_angle, motor_5_angle,
             tp, tr, side, J, J_l2pr, L, flag);
        double motor_4_v = 0.0;
        double motor_5_v = 0.0;
        velocity_pr2motor(J,vp,vr,motor_4_v,motor_5_v);
        torque_pr2motor(J,tq_p,tq_r,motor_4_tff,motor_5_tff);
        //printf("motor_4_angle = %f, motor_5_angle = %f/n",motor_4_angle,motor_5_angle);
        return std::make_tuple(motor_4_angle, motor_5_angle, motor_4_v, motor_5_v, -motor_4_tff, -motor_5_tff); //real motor
    }

std::tuple<double, double, double, double, double, double> 
    ParallelMechanism::jointToMotorRight(double tr, double tp, double vr, double vp,
                     double tq_r, double tq_p){
        int side = 2;
        double motor_10_angle = 0.0;
        double motor_11_angle = 0.0;
        double motor_10_tff = 0.0;
        double motor_11_tff = 0.0;
        Eigen::Matrix2d J;
        Eigen::Matrix2d J_l2pr;
        Eigen::Vector2d L;
        int flag = 0;
        ik(tp,tr,last_motor_10,last_motor_11,side,motor_10_angle,motor_11_angle);
        last_motor_10 = motor_10_angle;
        last_motor_11 = motor_11_angle;
        jac(motor_10_angle, motor_11_angle,
             tp, tr, side, J, J_l2pr, L, flag);
        torque_pr2motor(J,tq_p,tq_r,motor_10_tff,motor_11_tff);
        
        return std::make_tuple(0.0, 0.0, 0.0, 0.0, -motor_10_tff, -motor_11_tff); //real motor
    }

std::tuple<double, double, double, double, double, double> 
    ParallelMechanism::jointToMotorW(double tr, double tp, double vr, double vp,
                     double tq_r, double tq_p){
        int side = 3;
        double motor_13_angle = 0.0;
        double motor_14_angle = 0.0;
        double motor_13_tff = 0.0;
        double motor_14_tff = 0.0;
        Eigen::Matrix2d J;
        Eigen::Matrix2d J_l2pr;
        Eigen::Vector2d L;
        int flag = 0;
        ik(tp,tr,last_motor_13,last_motor_14,side,motor_13_angle,motor_14_angle);
        last_motor_13 = motor_13_angle;
        last_motor_14 = motor_14_angle;
        jac(motor_13_angle, motor_14_angle,
             tp, tr, side, J, J_l2pr, L, flag);
        torque_pr2motor(J,tq_p,tq_r,motor_13_tff,motor_14_tff);
        
        // return std::make_tuple(0.0, 0.0, 0.0, 0.0, motor_13_tff, motor_14_tff);
        return std::make_tuple(motor_13_angle, motor_14_angle, 0.0, 0.0, -motor_14_tff, -motor_13_tff); //real motor
    }

double last_pitch_l = 0;
double last_roll_l = 0;
double last_pitch_r = 0;
double last_roll_r = 0;
double last_pitch_w = 0;
double last_roll_w = 0;

std::tuple<double, double, double, double, double, double> 
    ParallelMechanism::motorToJointAnkle(int side, double t1, double t2, double v1, double v2,
                      double tq1, double tq2, double& last_pitch, double& last_roll){
        double pitch = 0.0;
        double roll = 0.0;
        double pitch_v = 0.0;
        double roll_v = 0.0;
        double pitch_t = 0.0;
        double roll_t = 0.0;
        Eigen::Matrix2d J;
        Eigen::Matrix2d J_l2pr;
        Eigen::Vector2d L;
        int flag = 0;
        if (side == 1){
            t1 = t1+0.261;
            t2 = t2+M_PI-0.261;
        }
        else{
            t1 = t1+M_PI-0.261;
            t2 = t2+0.261;
        }
        fk(t1, t2, last_pitch, last_roll, side, pitch, roll);
        last_pitch = pitch;
        last_roll = roll;
        jac(t1, t2, pitch, roll, side, J, J_l2pr, L, flag);
        velocity_motor2pr(J, v1, v2, pitch_v, roll_v);
        torque_motor2pr(J, tq1, tq2, pitch_t, roll_t);
    
        return std::make_tuple(roll, pitch, roll_v, pitch_v, roll_t, pitch_t); //urdf
    }

std::tuple<double, double, double, double, double, double> 
    ParallelMechanism::motorToJointLeft(double t1, double t2, double v1, double v2,
                      double tq1, double tq2){
        return motorToJointAnkle(1, t1, t2, v1, v2, tq1, tq2, last_pitch_l, last_roll_l);
    }

std::tuple<double, double, double, double, double, double> 
    ParallelMechanism::motorToJointRight(double t1, double t2, double v1, double v2,
                      double tq1, double tq2){
        return motorToJointAnkle(2, t1, t2, v1, v2, tq1, tq2, last_pitch_r, last_roll_r);
    }

std::tuple<double, double, double, double, double, double> 
    ParallelMechanism::motorToJointW(double t1, double t2, double v1, double v2,
                      double tq1, double tq2){
        return motorToJointWaist(t1, t2, v1, v2, tq1, tq2, last_pitch_w, last_roll_w);
    }

std::tuple<double, double, double, double, double, double>
    ParallelMechanism::motorToJointWaist(double t1, double t2, double v1, double v2,
                      double tq1, double tq2, double& last_pitch, double& last_roll){
        int side = 3;
        double pitch_w = 0.0;
        double roll_w = 0.0;
        double pitch_w_v = 0.0;
        double roll_w_v = 0.0;
        double pitch_w_t = 0.0;
        double roll_w_t = 0.0;
        Eigen::Matrix2d J;
        Eigen::Matrix2d J_l2pr;
        Eigen::Vector2d L;
        int flag = 0;
        t1 = t1 - 0.084;
        t2 = t2+M_PI + 0.084;
        fk(t1, t2, last_pitch, last_roll, side, pitch_w, roll_w);
        last_pitch = pitch_w;
        last_roll = roll_w;
        jac(t1, t2, pitch_w, roll_w, side, J, J_l2pr, L, flag);
        velocity_motor2pr(J, v1, v2, pitch_w_v, roll_w_v);
        torque_motor2pr(J, tq1, tq2, pitch_w_t, roll_w_t);
    
        return std::make_tuple(roll_w, pitch_w, roll_w_v, pitch_w_v, roll_w_t, pitch_w_t); //urdf
    }


#ifndef PARALLEL_MECHANISM_H
#define PARALLEL_MECHANISM_H

#include <Eigen/Dense>
#include <cmath>

using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::Matrix2d;
using Eigen::Vector2d;

// struct JointData {
//     double pos_ = 0.0;
//     double vel_ = 0.0;
//     double tau_ = 0.0;
    
//     JointData() = default;
//     JointData(double pos, double vel, double tau) 
//         : pos_(pos), vel_(vel), tau_(tau) {}
// };

class ParallelMechanism
{
public:
    Eigen::Matrix<double, 3, 3> motor_conter_1;
    Eigen::Matrix<double, 3, 3> motor_conter_2;
    Eigen::Matrix<double, 3, 3> S_down_1;
    Eigen::Matrix<double, 3, 3> S_down_2;

    const Eigen::Vector3d crank{0.0225, 0.0225, 0.0245};
    const Eigen::Vector3d rod_1{0.24601, 0.24601, 0.055};
    const Eigen::Vector3d rod_2{0.1851, 0.1851, 0.055};

    const Vector3d s{1, 0, 0};
    const Vector3d y{0, 1, 0};

    ParallelMechanism();

public:
    Matrix3d Ry(double theta) const;
    Matrix3d Rx(double theta) const;
    Matrix3d Ryx(double pitch, double roll) const;
    Matrix3d VectoT(const Vector3d& vec) const;

    int sincos(double A, double B, double C, double& theta, double& fai) const;

    int ik(double pitch_angle, double roll_angle,
           double motor_1_angle_bef, double motor_2_angle_bef,
           int side, double& motor_1_angle, double& motor_2_angle);

    void jac(double motor_1_angle, double motor_2_angle,
             double pitch_angle, double roll_angle, int side,
             Eigen::Matrix2d& J, Eigen::Matrix2d& J_l2pr,
             Eigen::Vector2d& L, int& flag);

    void fk(double motor_1_angle, double motor_2_angle,
            double pitch_angle_bef, double roll_angle_bef, int side,
            double& pitch_angle, double& roll_angle);

    void velocity_pr2motor(const Eigen::Matrix2d& J,
                           double pitch_vel, double roll_vel,
                           double& m1_vel, double& m2_vel);

    void velocity_motor2pr(const Eigen::Matrix2d& J,
                           double m1_vel, double m2_vel,
                           double& pitch_vel, double& roll_vel);
    
    void torque_pr2motor(const Eigen::Matrix2d& J,
                         double pitch_tau, double roll_tau,
                         double& m1_tau, double& m2_tau);

    void torque_motor2pr(const Eigen::Matrix2d& J,
                         double m1_tau, double m2_tau,
                         double& pitch_tau, double& roll_tau);

    std::tuple<double, double, double, double, double, double> 
    jointToMotorLeft(double tr, double tp, double vr, double vp,
                     double tq_r, double tq_p);
    std::tuple<double, double, double, double, double, double> 
    jointToMotorRight(double tr, double tp, double vr, double vp,
                     double tq_r, double tq_p);
    std::tuple<double, double, double, double, double, double> 
    jointToMotorW(double tr, double tp, double vr, double vp,
                     double tq_r, double tq_p);
    std::tuple<double, double, double, double, double, double> 
    motorToJointLeft(double t1, double t2, double v1, double v2,
                      double tq1, double tq2);
    std::tuple<double, double, double, double, double, double> 
    motorToJointRight(double t1, double t2, double v1, double v2,
                      double tq1, double tq2);
    std::tuple<double, double, double, double, double, double> 
    motorToJointW(double t1, double t2, double v1, double v2,
                      double tq1, double tq2);
    // 踝正解, 与 motorToJointLeft(side=1)/motorToJointRight(side=2) 相同, 但迭代初值由调用者保存,
    // 不读写全局 last_* 状态, 可在其它线程中使用独立实例调用
    std::tuple<double, double, double, double, double, double> 
    motorToJointAnkle(int side, double t1, double t2, double v1, double v2,
                      double tq1, double tq2, double& last_pitch, double& last_roll);
    // 腰正解, 与 motorToJointW 相同, 迭代初值由调用者保存(同上, 可在其它线程中使用独立实例调用)
    std::tuple<double, double, double, double, double, double>
    motorToJointWaist(double t1, double t2, double v1, double v2,
                      double tq1, double tq2, double& last_pitch, double& last_roll);
};

#endif

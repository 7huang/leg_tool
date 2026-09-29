#ifndef ANKEL_SOLVER_H
#define ANKEL_SOLVER_H

#include <iostream>
#include <cmath>
#include <tuple>
#include <vector>
#include <stdexcept>
#include <Eigen/Dense>

using namespace Eigen;

class AnkleParallelMechanism {
private:
    double ax_, cx_, l_bar_, l_y_, l_rod1_, l_rod2_, l_2_;
    Vector3d A1_, A2_;
    Vector3d C1_local_, C2_local_;
    double alpha1_zero_, alpha2_zero_;
    Vector3d O_;

    mutable double cached_roll_ = std::nan("");
    mutable double cached_pitch_ = std::nan("");
    mutable Matrix2d cached_J_inv_;
    
    Matrix3d rotationX(double theta) const;
    Matrix3d rotationY(double theta) const;
    Vector3d rotatePoint(const Vector3d& point, double roll, double pitch) const;
    Vector3d getBFromAlpha(const Vector3d& A, double alpha, int side) const;
    std::pair<Vector3d, double> findBFromGeometry(const Vector3d& A, const Vector3d& C, 
                                                   double rod_length, int side) const;
    
public:
    AnkleParallelMechanism();
    
    std::pair<double, double> inverseKinematicsPosition(double roll, double pitch) const;
    Matrix2d getVelocityJacobianAnalytical(double roll, double pitch) const;
    std::pair<double, double> inverseKinematicsVelocity(double roll, double pitch, 
                                                         double droll, double dpitch) const;
    std::pair<double, double> forwardKinematicsVelocity(double roll, double pitch,
                                                         double dalpha1, double dalpha2) const;
    std::pair<double, double> inverseKinematicsTorque(double roll, double pitch,
                                                       double tau_roll, double tau_pitch) const;
    std::pair<double, double> forwardKinematicsTorque(double roll, double pitch,
                                                       double tau1, double tau2) const;
    std::pair<double, double> forwardKinematicsPosition(double alpha1, double alpha2,
                                                         const Vector2d& guess = Vector2d(0, 0)) const;
};

struct JointData {
    double pos_ = 0.0;
    double vel_ = 0.0;
    double tau_ = 0.0;
    
    JointData() = default;
    JointData(double pos, double vel, double tau) 
        : pos_(pos), vel_(vel), tau_(tau) {}
};

class FastAnkelSolver {
private:
    AnkleParallelMechanism left_mechanism_;
    AnkleParallelMechanism right_mechanism_;
    
public:
    mutable double last_ankel_pitch_l = 0.0;
    mutable double last_ankel_roll_l = 0.0;
    mutable double last_ankel_pitch_r = 0.0;
    mutable double last_ankel_roll_r = 0.0;
    FastAnkelSolver(double r_l);
    
    std::tuple<double, double, double, double, double, double> 
    motorToJointLeft(double t1, double t2, double v1, double v2, 
                     double tq1, double tq2) const;
    
    std::tuple<double, double, double, double, double, double> 
    motorToJointRight(double t1, double t2, double v1, double v2,
                      double tq1, double tq2) const;
    
    std::tuple<double, double, double, double, double, double> 
    jointToMotorLeft(double tp, double tr, double vp, double vr,
                     double tq_p, double tq_r) const;
    
    std::tuple<double, double, double, double, double, double> 
    jointToMotorRight(double tp, double tr, double vp, double vr,
                      double tq_p, double tq_r) const;
    
    std::vector<JointData> motorToJointAll(const std::vector<JointData>& joint_data) const;
    
    void jointToMotorAll(std::vector<double>& q_d, std::vector<double>& v_d,
                        std::vector<double>& tq_d) const;
};

class WaistParallelMechanism {
private:
    double ax_, cx_, l_bar_, l_y_, l_rod1_, l_rod2_, l_2_;
    Vector3d A1_, A2_;
    Vector3d C1_local_, C2_local_;
    double alpha1_zero_, alpha2_zero_;
    Vector3d O_;

    mutable double cached_roll_ = std::nan("");
    mutable double cached_pitch_ = std::nan("");
    mutable Matrix2d cached_J_inv_;
    
    Matrix3d rotationX(double theta) const;
    Matrix3d rotationY(double theta) const;
    Vector3d rotatePoint(const Vector3d& point, double roll, double pitch) const;
    Vector3d getBFromAlpha(const Vector3d& A, double alpha, int side) const;
    std::pair<Vector3d, double> findBFromGeometry(const Vector3d& A, const Vector3d& C, 
                                                   double rod_length, int side) const;
    
public:
    WaistParallelMechanism();
    
    std::pair<double, double> inverseKinematicsPosition(double roll, double pitch) const;
    Matrix2d getVelocityJacobianAnalytical(double roll, double pitch) const;
    std::pair<double, double> inverseKinematicsVelocity(double roll, double pitch, 
                                                         double droll, double dpitch) const;
    std::pair<double, double> forwardKinematicsVelocity(double roll, double pitch,
                                                         double dalpha1, double dalpha2) const;
    std::pair<double, double> inverseKinematicsTorque(double roll, double pitch,
                                                       double tau_roll, double tau_pitch) const;
    std::pair<double, double> forwardKinematicsTorque(double roll, double pitch,
                                                       double tau1, double tau2) const;
    std::pair<double, double> forwardKinematicsPosition(double alpha1, double alpha2,
                                                         const Vector2d& guess = Vector2d(0, 0)) const;
};

class FastWaistSolver {
private:
    WaistParallelMechanism waist_mechanism_;
    
public:
    mutable double last_waist_pitch = 0.0;
    mutable double last_waist_roll = 0.0;
    FastWaistSolver(double r_l);
    
    std::tuple<double, double, double, double, double, double> 
    motorToJointW(double t1, double t2, double v1, double v2, 
                     double tq1, double tq2) const;
    
    std::tuple<double, double, double, double, double, double> 
    jointToMotorW(double tp, double tr, double vp, double vr,
                     double tq_p, double tq_r) const;
    
    std::vector<JointData> motorToJointAll(const std::vector<JointData>& joint_data) const;
    
    void jointToMotorAll(std::vector<double>& q_d, std::vector<double>& v_d,
                        std::vector<double>& tq_d) const;
};

void testMechanism();
#endif
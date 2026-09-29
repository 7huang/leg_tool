#include "close_chain_mapping.h"
#include <unistd.h>
using namespace Eigen;
double last_alpha1 = 0;
double last_alpha2 = 0;

    
    Matrix3d AnkleParallelMechanism::rotationX(double theta) const {
        Matrix3d R;
        R << 1, 0, 0,
             0, cos(theta), -sin(theta),
             0, sin(theta), cos(theta);
        return R;
    }
    
    Matrix3d AnkleParallelMechanism::rotationY(double theta) const {
        Matrix3d R;
        R << cos(theta), 0, sin(theta),
                      0, 1, 0,
             -sin(theta), 0, cos(theta);
        return R;
}

    Vector3d AnkleParallelMechanism::rotatePoint(const Vector3d& point, double roll, double pitch) const {
        Vector3d point_local = point;
        point_local(2) -= l_2_;

        Matrix3d Rx = rotationX(roll);
        Vector3d point_rotated = Rx * point_local;
        
        Matrix3d Ry = rotationY(pitch);
        point_rotated = Ry * point_rotated;
        
        point_rotated(2) += l_2_;
        return point_rotated;
}

    Vector3d AnkleParallelMechanism::getBFromAlpha(const Vector3d& A, double alpha, int side) const {
        Vector3d B_rel(0, side * l_bar_ * cos(alpha), side * l_bar_ * sin(alpha));
        return A + B_rel;
    }
    
    std::pair<Vector3d, double> AnkleParallelMechanism::findBFromGeometry(const Vector3d& A, const Vector3d& C, 
                                                   double rod_length, int side) const {
        double Ax = A(0), Ay = A(1), Az = A(2);
        double Cx = C(0), Cy = C(1), Cz = C(2);
        double Bx = Ax;
        
        double R2_sq = rod_length * rod_length - (Ax - Cx) * (Ax - Cx);
        if (R2_sq < 0) {
            printf("R2_sq = %f\n", R2_sq);
            return {Vector3d::Zero(), std::nan("")};
        }
        double R2 = std::sqrt(R2_sq);
        
        double d = std::sqrt((Cy - Ay) * (Cy - Ay) + (Cz - Az) * (Cz - Az));
        if (d > l_bar_ + R2 + 0.004 || d < std::abs(l_bar_ - R2) - 0.004) {
            printf("d = %f\n", d);
            printf("l_bar_ + R2 = %f\n", l_bar_ + R2);
            printf("l_bar_ - R2 = %f\n", l_bar_ - R2);
            return {Vector3d::Zero(), std::nan("")};
        }
        
        double dx = Cy - Ay;
        double dz = Cz - Az;
        double a = (l_bar_ * l_bar_ - R2 * R2 + d * d) / (2 * d);
        double h2 = l_bar_ * l_bar_ - a * a;
        double h = std::sqrt(std::max(0.0, h2));
        
        double ux, uz;
        if (d > 1e-8) {
            ux = dx / d;
            uz = dz / d;
        } else {
            ux = 1.0;
            uz = 0.0;
        }
        
        double Px = Ay + a * ux;
        double Pz = Az + a * uz;
        double vx = -uz;
        double vz = ux;
        
        double By1 = Px + h * vx;
        double Bz1 = Pz + h * vz;
        double By2 = Px - h * vx;
        double Bz2 = Pz - h * vz;
        
        Vector3d B;
        if (side == 1) {
            B = (By1 > Ay) ? Vector3d(Bx, By1, Bz1) : Vector3d(Bx, By2, Bz2);
        } else {
            B = (By1 < Ay) ? Vector3d(Bx, By1, Bz1) : Vector3d(Bx, By2, Bz2);
        }
        
        Vector3d B_rel = B - A;
        double alpha = std::atan2(B_rel(2), B_rel(1));
        return {B, alpha};
    }
    

    AnkleParallelMechanism::AnkleParallelMechanism() : ax_(0.04475), cx_(0.03175), l_bar_(0.0225), 
                               l_y_(0.02173), l_rod1_(0.24601), l_rod2_(0.18509), l_2_(0.0) {
        // Initialize A1 and A2
        A1_ = Vector3d(-ax_, 0, 0.24467 - 0.00582);
        A2_ = Vector3d(-ax_, 0, 0.18364 - 0.00582);
        
        // Initialize C points in local coordinates
        C1_local_ = Vector3d(-cx_, l_y_, -0.001);
        C2_local_ = Vector3d(-cx_, -l_y_, -0.001);
        
        // Calculate zero position motor angles
        Vector3d B1_zero(-ax_, l_y_, 0.24467);
        Vector3d B2_zero(-ax_, -l_y_, 0.18364);
        Vector3d B1_rel_zero = B1_zero - A1_;
        Vector3d B2_rel_zero = B2_zero - A2_;
        alpha1_zero_ = std::atan2(B1_rel_zero(2), B1_rel_zero(1));
        alpha2_zero_ = std::atan2(B2_rel_zero(2), B2_rel_zero(1));
        
        O_ = Vector3d(0, 0, 0);
    }
    
    // Position inverse kinematics
    std::pair<double, double> AnkleParallelMechanism::inverseKinematicsPosition(double roll, double pitch) const {        
        // printf("p = %f, r = %f\n" , pitch, roll);
        Vector3d C1 = rotatePoint(C1_local_, roll, pitch);
        Vector3d C2 = rotatePoint(C2_local_, roll, pitch);
        
        auto [B1, alpha1_full] = findBFromGeometry(A1_, C1, l_rod1_, 1);
        auto [B2, alpha2_full] = findBFromGeometry(A2_, C2, l_rod2_, -1);
        
        if (std::isnan(alpha1_full) || std::isnan(alpha2_full)) {
            // throw std::runtime_error("Position inverse kinematics has no solution");
            printf("nan detected, pitch = %f, roll = %f\n", pitch, roll);
            return {last_alpha1, last_alpha2};
        }
        last_alpha1 = alpha1_full - alpha1_zero_;
        last_alpha2 = alpha2_full - alpha2_zero_ + 3.14159;
        return {alpha1_full - alpha1_zero_, alpha2_full - alpha2_zero_ + 3.14159};
    }
    
    // Get velocity Jacobian (analytical via finite difference)
    Matrix2d AnkleParallelMechanism::getVelocityJacobianAnalytical(double roll, double pitch) const {
        
        const double eps = 1e-4;
        if (fabs(roll - cached_roll_) < eps && fabs(pitch - cached_pitch_) < eps)
            return cached_J_inv_;
        
        const double delta = 1e-5;
        Matrix2d J_inv;
        
        // Current motor angles
        auto [alpha1, alpha2] = inverseKinematicsPosition(roll, pitch);
        
        // Partial derivative with respect to roll
        auto [alpha1_r, alpha2_r] = inverseKinematicsPosition(roll + delta, pitch);
        J_inv(0, 0) = (alpha1_r - alpha1) / delta;
        J_inv(1, 0) = (alpha2_r - alpha2) / delta;
        
        // Partial derivative with respect to pitch
        auto [alpha1_p, alpha2_p] = inverseKinematicsPosition(roll, pitch + delta);
        J_inv(0, 1) = (alpha1_p - alpha1) / delta;
        J_inv(1, 1) = (alpha2_p - alpha2) / delta;

        cached_roll_ = roll;
        cached_pitch_ = pitch;
        cached_J_inv_ = J_inv;
        
        return J_inv;
    }
    
    // Velocity inverse kinematics
    std::pair<double, double> AnkleParallelMechanism::inverseKinematicsVelocity(double roll, double pitch, 
                                                         double droll, double dpitch) const {
        Matrix2d J_inv = getVelocityJacobianAnalytical(roll, pitch);
        Vector2d q_dot(droll, dpitch);
        Vector2d dalpha = J_inv * q_dot;
        return {dalpha(0), dalpha(1)};
    }
    
    // Velocity forward kinematics
    std::pair<double, double> AnkleParallelMechanism::forwardKinematicsVelocity(double roll, double pitch,
                                                         double dalpha1, double dalpha2) const {
        Matrix2d J_inv = getVelocityJacobianAnalytical(roll, pitch);
        Vector2d dalpha(dalpha1, dalpha2);
        Vector2d q_dot = J_inv.completeOrthogonalDecomposition().pseudoInverse() * dalpha;
        return {q_dot(0), q_dot(1)};
    }
    
    // Torque inverse kinematics
    std::pair<double, double> AnkleParallelMechanism::inverseKinematicsTorque(double roll, double pitch,
                                                       double tau_roll, double tau_pitch) const {
        Matrix2d J_inv = getVelocityJacobianAnalytical(roll, pitch);
        Vector2d tau_ankle(tau_roll, tau_pitch);
        Vector2d tau_motor = J_inv.transpose().completeOrthogonalDecomposition().pseudoInverse() * tau_ankle;
        return {tau_motor(0), tau_motor(1)};
    }
    
    // Torque forward kinematics
    std::pair<double, double> AnkleParallelMechanism::forwardKinematicsTorque(double roll, double pitch,
                                                       double tau1, double tau2) const {
        Matrix2d J_inv = getVelocityJacobianAnalytical(roll, pitch);
        Vector2d tau_motor(tau1, tau2);
        Vector2d tau_ankle = J_inv.transpose() * tau_motor;
        return {tau_ankle(0), tau_ankle(1)};
    }
    
    // Position forward kinematics using BFGS-inspired optimization
    // Position forward kinematics using BFGS-inspired optimization
    std::pair<double, double> AnkleParallelMechanism::forwardKinematicsPosition(double alpha1, double alpha2,
                                                        const Vector2d& guess) const {
        double alpha1_full = alpha1 + alpha1_zero_;
        double alpha2_full = alpha2 + alpha2_zero_;
        
        Vector3d B1 = getBFromAlpha(A1_, alpha1_full, 1);
        Vector3d B2 = getBFromAlpha(A2_, alpha2_full, -1);
        
        // Use BFGS-inspired optimization
        Vector2d q = guess;
        Vector2d q_old = q;  // 保存上一次的值
        const double tolerance = 1e-6;
        const int max_iterations = 5;
        
        // BFGS variables
        Matrix2d H = Matrix2d::Identity();
        Vector2d grad_old;
        double error_old;
        
        // Initial error and gradient
        Vector3d C1 = rotatePoint(C1_local_, q(0), q(1));
        Vector3d C2 = rotatePoint(C2_local_, q(0), q(1));
        double err1 = (B1 - C1).norm() - l_rod1_;
        double err2 = (B2 - C2).norm() - l_rod2_;
        double error = err1 * err1 + err2 * err2;
        
        for (int iter = 0; iter < max_iterations; ++iter) {
            if (error < tolerance) {
                break;
            }
            
            // Compute gradient using finite differences
            Vector2d gradient;
            double delta = 1e-4;
            
            // df/dq0
            Vector3d C1_dq0 = rotatePoint(C1_local_, q(0) + delta, q(1));
            Vector3d C2_dq0 = rotatePoint(C2_local_, q(0) + delta, q(1));
            double err1_dq0 = (B1 - C1_dq0).norm() - l_rod1_;
            double err2_dq0 = (B2 - C2_dq0).norm() - l_rod2_;
            double error_dq0 = err1_dq0 * err1_dq0 + err2_dq0 * err2_dq0;
            gradient(0) = (error_dq0 - error) / delta;
            
            // df/dq1
            Vector3d C1_dq1 = rotatePoint(C1_local_, q(0), q(1) + delta);
            Vector3d C2_dq1 = rotatePoint(C2_local_, q(0), q(1) + delta);
            double err1_dq1 = (B1 - C1_dq1).norm() - l_rod1_;
            double err2_dq1 = (B2 - C2_dq1).norm() - l_rod2_;
            double error_dq1 = err1_dq1 * err1_dq1 + err2_dq1 * err2_dq1;
            gradient(1) = (error_dq1 - error) / delta;
            
            // BFGS update (只在第二次迭代及以后进行)
            if (iter > 0) {
                Vector2d s = q - q_old;           // 步长（当前位置变化）
                Vector2d y = gradient - grad_old;  // 梯度变化
                
                double sy = y.dot(s);
                if (sy > 1e-10) {  // 避免除零
                    double rho = 1.0 / sy;
                    Matrix2d I = Matrix2d::Identity();
                    H = (I - rho * s * y.transpose()) * H * (I - rho * y * s.transpose()) + rho * s * s.transpose();
                }
            }
            
            // 保存当前值供下次迭代使用
            q_old = q;
            grad_old = gradient;
            error_old = error;
            
            // 更新状态
            Vector2d delta_q = -H * gradient;
            
            // Line search (simple backtracking)
            // double alpha_step = 1.0;
            double alpha_step = 10.0 * pow(0.75, iter);
            double c = 0.5;
            double rho_line = 0.5;
            
            Vector2d q_new = q + alpha_step * delta_q;
            C1 = rotatePoint(C1_local_, q_new(0), q_new(1));
            C2 = rotatePoint(C2_local_, q_new(0), q_new(1));
            err1 = (B1 - C1).norm() - l_rod1_;
            err2 = (B2 - C2).norm() - l_rod2_;
            double error_new = err1 * err1 + err2 * err2;
            
            while (error_new > error + c * alpha_step * gradient.dot(delta_q) && alpha_step > 1e-8) {
                alpha_step *= rho_line;
                q_new = q + alpha_step * delta_q;
                C1 = rotatePoint(C1_local_, q_new(0), q_new(1));
                C2 = rotatePoint(C2_local_, q_new(0), q_new(1));
                err1 = (B1 - C1).norm() - l_rod1_;
                err2 = (B2 - C2).norm() - l_rod2_;
                error_new = err1 * err1 + err2 * err2;
            }
            
            // 更新
            q = q_new;
            error = error_new;
        }
        
        return {q(0), q(1)};
    }



    FastAnkelSolver::FastAnkelSolver(double r_l) {
        // The r_l parameter might be used for some configuration
        // Currently not used as the mechanisms are symmetric
        (void)r_l; // Suppress unused parameter warning
    }
    
    std::tuple<double, double, double, double, double, double> 
    FastAnkelSolver::motorToJointLeft(double t1, double t2, double v1, double v2, 
                     double tq1, double tq2) const {
        // t1, t2 are motor positions (alpha1, alpha2)
        // Convert from motor to joint angles (position forward kinematics)
        double roll_guess = last_ankel_roll_l;
        double pitch_guess = last_ankel_pitch_l;
        t2 += 3.14159;
        auto [roll, pitch] = left_mechanism_.forwardKinematicsPosition(t1, t2, Vector2d(roll_guess, pitch_guess));
        last_ankel_roll_l = roll;
        last_ankel_pitch_l = pitch;

        // Velocity forward kinematics
        auto [vr, vp] = left_mechanism_.forwardKinematicsVelocity(roll, pitch, v1, v2);

        // Torque forward kinematics
        auto [tq_r, tq_p] = left_mechanism_.forwardKinematicsTorque(roll, pitch, tq1, tq2);
        // double vp,vr,tq_p,tq_r = 0.0;
        return std::make_tuple(roll, pitch, vr, vp, tq_r, tq_p);
    }
    
    std::tuple<double, double, double, double, double, double> 
    FastAnkelSolver::motorToJointRight(double t1, double t2, double v1, double v2,
                      double tq1, double tq2) const {
        // For right side, we might need to mirror the angles
        double roll_guess = last_ankel_roll_r;
        double pitch_guess = last_ankel_pitch_r;
        t2 += 3.14159;

        auto [roll, pitch] = right_mechanism_.forwardKinematicsPosition(t1, t2, Vector2d(roll_guess, pitch_guess));
        last_ankel_roll_r = roll;
        last_ankel_pitch_r = pitch;

        auto [vr, vp] = right_mechanism_.forwardKinematicsVelocity(roll, pitch, v1, v2);
        //printf("right pitch = %f, roll = %f, vp = %f, vr = %f\n", pitch, roll, vp, vr);
        auto [tq_r, tq_p] = right_mechanism_.forwardKinematicsTorque(roll, pitch, tq1, tq2);
        
        return std::make_tuple(roll, pitch, vr, vp, tq_r, tq_p);
    }
    
    std::tuple<double, double, double, double, double, double> 
    FastAnkelSolver::jointToMotorLeft(double tr, double tp, double vr, double vp,
                     double tq_r, double tq_p) const {

        double alpha1=0.0, alpha2=0.0, v1=0.0, v2=0.0;
        auto [tq1, tq2] = left_mechanism_.inverseKinematicsTorque(tr, tp, tq_r, tq_p);
        
        return std::make_tuple(alpha1, alpha2, v1, v2, tq1, tq2);
    }
    
    std::tuple<double, double, double, double, double, double> 
    FastAnkelSolver::jointToMotorRight(double tr, double tp, double vr, double vp,
                      double tq_r, double tq_p) const {
                        
        double alpha1=0.0, alpha2=0.0, v1=0.0, v2=0.0;
        auto [tq1, tq2] = right_mechanism_.inverseKinematicsTorque(tr, tp, tq_r, tq_p);
        
        return std::make_tuple(alpha1, alpha2, v1, v2, tq1, tq2);
    }
    
    std::vector<JointData> FastAnkelSolver::motorToJointAll(const std::vector<JointData>& joint_data) const {
        if (joint_data.size() < 12) {
            throw std::runtime_error("joint_data size must be at least 12");
        }
        
        std::vector<JointData> result = joint_data;
        
        // Left side (indices 4 and 5 - motors 5 and 6)
        double tp, tr, vp, vr, tq_p, tq_r;
        std::tie(tr, tp, vr, vp, tq_r, tq_p) = motorToJointLeft(
            joint_data[4].pos_, joint_data[5].pos_,
            joint_data[4].vel_, joint_data[5].vel_,
            joint_data[4].tau_, joint_data[5].tau_
        );
        
        result[4].pos_ = tp;
        result[4].vel_ = vp;
        result[4].tau_ = tq_p;
        
        result[5].pos_ = tr;
        result[5].vel_ = vr;
        result[5].tau_ = tq_r;
        
        // Right side (indices 10 and 11 - motors 11 and 12)
        std::tie(tr, tp, vr, vp, tq_r, tq_p) = motorToJointRight(
            joint_data[10].pos_, joint_data[11].pos_,
            joint_data[10].vel_, joint_data[11].vel_,
            joint_data[10].tau_, joint_data[11].tau_
        );
        
        result[10].pos_ = tp;
        result[10].vel_ = vp;
        result[10].tau_ = tq_p;
        
        result[11].pos_ = tr;
        result[11].vel_ = vr;
        result[11].tau_ = tq_r;
        
        return result;
    }
    
    void FastAnkelSolver::jointToMotorAll(std::vector<double>& q_d, std::vector<double>& v_d,
                        std::vector<double>& tq_d) const {
        // This function should convert joint-space desired values to motor-space
        // Assuming q_d, v_d, tq_d are for joints 4,5 (left pitch/roll) and 10,11 (right pitch/roll)
        if (q_d.size() < 12 || v_d.size() < 12 || tq_d.size() < 12) {
            throw std::runtime_error("Input vectors must have size at least 12");
        }
        
        // Left side (indices 4 and 5)
        double alpha1, alpha2, v1, v2, tq1, tq2;
        std::tie(alpha1, alpha2, v1, v2, tq1, tq2) = jointToMotorLeft(
            q_d[5], q_d[4], v_d[5], v_d[4], tq_d[5], tq_d[4]
        );
        
        q_d[4] = alpha1;
        q_d[5] = alpha2;
        v_d[4] = v1;
        v_d[5] = v2;
        tq_d[4] = tq1;
        tq_d[5] = tq2;
        
        // Right side (indices 10 and 11)
        std::tie(alpha1, alpha2, v1, v2, tq1, tq2) = jointToMotorRight(
            q_d[11], q_d[10], v_d[11], v_d[10], tq_d[11], tq_d[10]
        );
        
        q_d[10] = alpha1;
        q_d[11] = alpha2;
        v_d[10] = v1;
        v_d[11] = v2;
        tq_d[10] = tq1;
        tq_d[11] = tq2;
    }


// Test function
void testMechanism() {
    FastAnkelSolver solver(0.0);
    AnkleParallelMechanism mech;
    
    std::cout << "============================================================" << std::endl;
    std::cout << "Ankle Parallel Mechanism Kinematics and Dynamics Test" << std::endl;
    std::cout << "============================================================" << std::endl;
    
    double test_roll = 0.0;
    double test_pitch = -0.6; // rad, about -34.4°
    
    std::cout << "\nTest pose: roll=" << test_roll * 180/M_PI << "°, pitch=" << test_pitch * 180/M_PI << "°" << std::endl;
    
    // 1. Position inverse kinematics
    auto [alpha1, alpha2] = mech.inverseKinematicsPosition(test_roll, test_pitch);
    std::cout << "\n[Position Inverse Kinematics]" << std::endl;
    std::cout << "  Motor angle 1: " << alpha1 * 180/M_PI << "°" << std::endl;
    std::cout << "  Motor angle 2: " << alpha2 * 180/M_PI << "°" << std::endl;
    
    // 2. Position forward kinematics verification
    auto [roll_fk, pitch_fk] = mech.forwardKinematicsPosition(alpha1, alpha2, Vector2d(test_roll, test_pitch));
    std::cout << "\n[Position Forward Kinematics Verification]" << std::endl;
    std::cout << "  Solved pose: roll=" << roll_fk * 180/M_PI << "°, pitch=" << pitch_fk * 180/M_PI << "°" << std::endl;
    std::cout << "  Error: roll=" << (roll_fk - test_roll) * 180/M_PI << "°, pitch=" << (pitch_fk - test_pitch) * 180/M_PI << "°" << std::endl;
    
    // 3. Velocity inverse kinematics
    double test_droll = 0.2;
    double test_dpitch = 0.8;
    auto [dalpha1, dalpha2] = mech.inverseKinematicsVelocity(test_roll, test_pitch, test_droll, test_dpitch);
    std::cout << "\n[Velocity Inverse Kinematics]" << std::endl;
    std::cout << "  Ankle angular velocity: droll=" << test_droll << " rad/s, dpitch=" << test_dpitch << " rad/s" << std::endl;
    std::cout << "  Motor angular velocity 1: " << dalpha1 << " rad/s" << std::endl;
    std::cout << "  Motor angular velocity 2: " << dalpha2 << " rad/s" << std::endl;
    
    // 4. Velocity forward kinematics verification
    auto [droll_fk, dpitch_fk] = mech.forwardKinematicsVelocity(test_roll, test_pitch, dalpha1, dalpha2);
    std::cout << "\n[Velocity Forward Kinematics Verification]" << std::endl;
    std::cout << "  Solved angular velocity: droll=" << droll_fk << " rad/s, dpitch=" << dpitch_fk << " rad/s" << std::endl;
    std::cout << "  Error: droll=" << (droll_fk - test_droll) << " rad/s, dpitch=" << (dpitch_fk - test_dpitch) << " rad/s" << std::endl;
    
    // 5. Torque inverse kinematics
    double test_tau_roll = 10.0;
    double test_tau_pitch = 5.0;
    auto [tau1, tau2] = mech.inverseKinematicsTorque(test_roll, test_pitch, test_tau_roll, test_tau_pitch);
    std::cout << "\n[Torque Inverse Kinematics]" << std::endl;
    std::cout << "  Ankle torque: tau_roll=" << test_tau_roll << " Nm, tau_pitch=" << test_tau_pitch << " Nm" << std::endl;
    std::cout << "  Motor torque 1: " << tau1 << " Nm" << std::endl;
    std::cout << "  Motor torque 2: " << tau2 << " Nm" << std::endl;
    
    // 6. Torque forward kinematics verification
    auto [tau_roll_fk, tau_pitch_fk] = mech.forwardKinematicsTorque(test_roll, test_pitch, tau1, tau2);
    std::cout << "\n[Torque Forward Kinematics Verification]" << std::endl;
    std::cout << "  Solved torque: tau_roll=" << tau_roll_fk << " Nm, tau_pitch=" << tau_pitch_fk << " Nm" << std::endl;
    std::cout << "  Error: tau_roll=" << (tau_roll_fk - test_tau_roll) << " Nm, tau_pitch=" << (tau_pitch_fk - test_tau_pitch) << " Nm" << std::endl;
    
    // 7. Jacobian matrix
    Matrix2d J_inv = mech.getVelocityJacobianAnalytical(test_roll, test_pitch);
    double cond_num = J_inv.jacobiSvd().singularValues()(0) / J_inv.jacobiSvd().singularValues()(1);
    std::cout << "\n[Jacobian Matrix]" << std::endl;
    std::cout << "  J_inv =" << std::endl;
    std::cout << "    [" << J_inv(0,0) << ", " << J_inv(0,1) << "]" << std::endl;
    std::cout << "    [" << J_inv(1,0) << ", " << J_inv(1,1) << "]" << std::endl;
    std::cout << "  Condition number: " << cond_num << std::endl;
    
    if (std::abs(droll_fk - test_droll) < 1e-6 && std::abs(dpitch_fk - test_dpitch) < 1e-6) {
        std::cout << "\n✓ Velocity forward kinematics accuracy OK" << std::endl;
    } else {
        std::cout << "\n✗ Velocity forward kinematics still has error" << std::endl;
    }
}
/////////////////////////////////////////////
/////////////////////////////////////////////
/////////////////////////////////////////////
/////////////////////////////////////////////
/////////////////////////////////////////////
/////////////////////////////////////////////
/////////////////////////////////////////////
/////////////////////////////////////////////
/////////////////////////////////////////////
/////////////////////////////////////////////
/////////////////////////////////////////////
/////////////////////////////////////////////

double last_alpha3 = 0;
double last_alpha4 = 0;

    
    Matrix3d WaistParallelMechanism::rotationX(double theta) const {
        Matrix3d R;
        R << 1, 0, 0,
             0, cos(theta), -sin(theta),
             0, sin(theta), cos(theta);
        return R;
    }
    
    Matrix3d WaistParallelMechanism::rotationY(double theta) const {
        Matrix3d R;
        R << cos(theta), 0, sin(theta),
             0, 1, 0,
             -sin(theta), 0, cos(theta);
        return R;
    }
    
    Vector3d WaistParallelMechanism::rotatePoint(const Vector3d& point, double roll, double pitch) const {
        Vector3d point_local = point;
        point_local(2) -= l_2_;
        
        Matrix3d Rx = rotationX(roll);
        Vector3d point_rotated = Rx * point_local;
        
        Matrix3d Ry = rotationY(pitch);
        point_rotated = Ry * point_rotated;
        
        point_rotated(2) += l_2_;
        return point_rotated;
    }
    
    Vector3d WaistParallelMechanism::getBFromAlpha(const Vector3d& A, double alpha, int side) const {
        if (side == 1){
            Vector3d B_rel(0, -side * l_bar_ * cos(alpha), -side * l_bar_ * sin(alpha));
            return A + B_rel;
        }
        if (side == -1){
            Vector3d B_rel(0, -side * l_bar_ * cos(alpha), -side * l_bar_ * sin(alpha));
            return A + B_rel;
        }
    }
    
    std::pair<Vector3d, double> WaistParallelMechanism::findBFromGeometry(const Vector3d& A, const Vector3d& C, 
                                                   double rod_length, int side) const {
        double Ax = A(0), Ay = A(1), Az = A(2);
        double Cx = C(0), Cy = C(1), Cz = C(2);
        double Bx = Ax;
        
        double R2_sq = rod_length * rod_length - (Ax - Cx) * (Ax - Cx);
        if (R2_sq < 0) {
            printf("R2_sq = %f\n", R2_sq);
            return {Vector3d::Zero(), std::nan("")};
        }
        double R2 = std::sqrt(R2_sq);
        
        double d = std::sqrt((Cy - Ay) * (Cy - Ay) + (Cz - Az) * (Cz - Az));
        if (d > l_bar_ + R2 + 0.004 || d < std::abs(l_bar_ - R2) - 0.004) {
            printf("d = %f\n", d);
            printf("l_bar_ + R2 = %f\n", l_bar_ + R2);
            printf("l_bar_ - R2 = %f\n", l_bar_ - R2);
            return {Vector3d::Zero(), std::nan("")};
        }
        
        double dx = Cy - Ay;
        double dz = Cz - Az;
        double a = (l_bar_ * l_bar_ - R2 * R2 + d * d) / (2 * d);
        double h2 = l_bar_ * l_bar_ - a * a;
        double h = std::sqrt(std::max(0.0, h2));
        
        double ux, uz;
        if (d > 1e-8) {
            ux = dx / d;
            uz = dz / d;
        } else {
            ux = 1.0;
            uz = 0.0;
        }
        
        double Px = Ay + a * ux;
        double Pz = Az + a * uz;
        double vx = -uz;
        double vz = ux;
        
        double By1 = Px + h * vx;
        double Bz1 = Pz + h * vz;
        double By2 = Px - h * vx;
        double Bz2 = Pz - h * vz;
        
        Vector3d B;
        if (side == 1) {
            B = (By1 > Ay) ? Vector3d(Bx, By1, Bz1) : Vector3d(Bx, By2, Bz2);
        } else {
            B = (By1 < Ay) ? Vector3d(Bx, By1, Bz1) : Vector3d(Bx, By2, Bz2);
        }
        
        Vector3d B_rel = B - A;
        double alpha = std::atan2(B_rel(2), B_rel(1));
        return {B, alpha};
    }
    

    WaistParallelMechanism::WaistParallelMechanism() : ax_(0.0435), cx_(0.0435), l_bar_(0.0245), 
                               l_y_(0.028), l_rod1_(0.055), l_rod2_(0.055), l_2_(0.0) {
        // Initialize A1 and A2
        A1_ = Vector3d(-ax_, 0.0375, 0.048);
        A2_ = Vector3d(-ax_, -0.0375, 0.048);
        
        // Initialize C points in local coordinates
        C1_local_ = Vector3d(-cx_, l_y_, -0.007);
        C2_local_ = Vector3d(-cx_, -l_y_, -0.007);
        
        // Calculate zero position motor angles
        Vector3d B1_zero(-ax_, l_y_, 0.046);
        Vector3d B2_zero(-ax_, -l_y_, 0.046);
        Vector3d B1_rel_zero = B1_zero - A1_;
        Vector3d B2_rel_zero = B2_zero - A2_;
        alpha1_zero_ = std::atan2(B1_rel_zero(2), B1_rel_zero(1));
        alpha2_zero_ = std::atan2(B2_rel_zero(2), B2_rel_zero(1));
        
        O_ = Vector3d(0, 0, 0);
    }
    
    // Position inverse kinematics
    std::pair<double, double> WaistParallelMechanism::inverseKinematicsPosition(double roll, double pitch) const {        
        // printf("p = %f, r = %f\n" , pitch, roll);
        Vector3d C1 = rotatePoint(C1_local_, roll, pitch);
        Vector3d C2 = rotatePoint(C2_local_, roll, pitch);
        
        auto [B1, alpha1_full] = findBFromGeometry(A1_, C1, l_rod1_, -1);
        auto [B2, alpha2_full] = findBFromGeometry(A2_, C2, l_rod2_, 1);
        
        if (std::isnan(alpha1_full) || std::isnan(alpha2_full)) {
            // throw std::runtime_error("Position inverse kinematics has no solution");
            printf("nan detected, pitch = %f, roll = %f\n", pitch, roll);
            return {last_alpha3, last_alpha4};
        }
        last_alpha3 = alpha1_full - alpha1_zero_+ 3.14159;
        last_alpha4 = alpha2_full - alpha2_zero_ ;
        return {alpha1_full - alpha1_zero_+ 3.14159, alpha2_full - alpha2_zero_};
    }
    
    // Get velocity Jacobian (analytical via finite difference)
    Matrix2d WaistParallelMechanism::getVelocityJacobianAnalytical(double roll, double pitch) const {
        
        if (roll == cached_roll_ && pitch == cached_pitch_) {
            // printf("using cached p&r\n");
            return cached_J_inv_;
        }
        
        const double delta = 1e-5;
        Matrix2d J_inv;
        
        // Current motor angles
        auto [alpha1, alpha2] = inverseKinematicsPosition(roll, pitch);
        
        // Partial derivative with respect to roll
        auto [alpha1_r, alpha2_r] = inverseKinematicsPosition(roll + delta, pitch);
        J_inv(0, 0) = (alpha1_r - alpha1) / delta;
        J_inv(1, 0) = (alpha2_r - alpha2) / delta;
        
        // Partial derivative with respect to pitch
        auto [alpha1_p, alpha2_p] = inverseKinematicsPosition(roll, pitch + delta);
        J_inv(0, 1) = (alpha1_p - alpha1) / delta;
        J_inv(1, 1) = (alpha2_p - alpha2) / delta;

        cached_roll_ = roll;
        cached_pitch_ = pitch;
        cached_J_inv_ = J_inv;
        
        return J_inv;
    }
    
    // Velocity inverse kinematics
    std::pair<double, double> WaistParallelMechanism::inverseKinematicsVelocity(double roll, double pitch, 
                                                         double droll, double dpitch) const {
        Matrix2d J_inv = getVelocityJacobianAnalytical(roll, pitch);
        Vector2d q_dot(droll, dpitch);
        Vector2d dalpha = J_inv * q_dot;
        return {dalpha(0), dalpha(1)};
    }
    
    // Velocity forward kinematics
    std::pair<double, double> WaistParallelMechanism::forwardKinematicsVelocity(double roll, double pitch,
                                                         double dalpha1, double dalpha2) const {
        Matrix2d J_inv = getVelocityJacobianAnalytical(roll, pitch);
        Vector2d dalpha(dalpha1, dalpha2);
        Vector2d q_dot = J_inv.completeOrthogonalDecomposition().pseudoInverse() * dalpha;
        return {q_dot(0), q_dot(1)};
    }
    
    // Torque inverse kinematics
    std::pair<double, double> WaistParallelMechanism::inverseKinematicsTorque(double roll, double pitch,
                                                       double tau_roll, double tau_pitch) const {
        Matrix2d J_inv = getVelocityJacobianAnalytical(roll, pitch);
        Vector2d tau_ankle(tau_roll, tau_pitch);
        Vector2d tau_motor = J_inv.transpose().completeOrthogonalDecomposition().pseudoInverse() * tau_ankle;
        return {tau_motor(0), tau_motor(1)};
    }
    
    // Torque forward kinematics
    std::pair<double, double> WaistParallelMechanism::forwardKinematicsTorque(double roll, double pitch,
                                                       double tau1, double tau2) const {
        Matrix2d J_inv = getVelocityJacobianAnalytical(roll, pitch);
        Vector2d tau_motor(tau1, tau2);
        Vector2d tau_ankle = J_inv.transpose() * tau_motor;
        return {tau_ankle(0), tau_ankle(1)};
    }
    
    // Position forward kinematics using BFGS-inspired optimization
    // Position forward kinematics using BFGS-inspired optimization
    std::pair<double, double> WaistParallelMechanism::forwardKinematicsPosition(double alpha1, double alpha2,
                                                        const Vector2d& guess) const {
        double alpha1_full = alpha1 + alpha1_zero_;
        double alpha2_full = alpha2 + alpha2_zero_;
        
        Vector3d B1 = getBFromAlpha(A1_, alpha1_full, 1);
        Vector3d B2 = getBFromAlpha(A2_, alpha2_full, -1);
        
        // Use BFGS-inspired optimization
        Vector2d q = guess;
        Vector2d q_old = q;  // 保存上一次的值
        const double tolerance = 1e-6;
        const int max_iterations = 5;
        
        // BFGS variables
        Matrix2d H = Matrix2d::Identity();
        Vector2d grad_old;
        double error_old;
        
        // Initial error and gradient
        Vector3d C1 = rotatePoint(C1_local_, q(0), q(1));
        Vector3d C2 = rotatePoint(C2_local_, q(0), q(1));
        double err1 = (B1 - C1).norm() - l_rod1_;
        double err2 = (B2 - C2).norm() - l_rod2_;
        double error = err1 * err1 + err2 * err2;
        
        for (int iter = 0; iter < max_iterations; ++iter) {
            if (error < tolerance) {
                break;
            }
            
            // Compute gradient using finite differences
            Vector2d gradient;
            double delta = 1e-4;
            
            // df/dq0
            Vector3d C1_dq0 = rotatePoint(C1_local_, q(0) + delta, q(1));
            Vector3d C2_dq0 = rotatePoint(C2_local_, q(0) + delta, q(1));
            double err1_dq0 = (B1 - C1_dq0).norm() - l_rod1_;
            double err2_dq0 = (B2 - C2_dq0).norm() - l_rod2_;
            double error_dq0 = err1_dq0 * err1_dq0 + err2_dq0 * err2_dq0;
            gradient(0) = (error_dq0 - error) / delta;
            
            // df/dq1
            Vector3d C1_dq1 = rotatePoint(C1_local_, q(0), q(1) + delta);
            Vector3d C2_dq1 = rotatePoint(C2_local_, q(0), q(1) + delta);
            double err1_dq1 = (B1 - C1_dq1).norm() - l_rod1_;
            double err2_dq1 = (B2 - C2_dq1).norm() - l_rod2_;
            double error_dq1 = err1_dq1 * err1_dq1 + err2_dq1 * err2_dq1;
            gradient(1) = (error_dq1 - error) / delta;
            
            // BFGS update (只在第二次迭代及以后进行)
            if (iter > 0) {
                Vector2d s = q - q_old;           // 步长（当前位置变化）
                Vector2d y = gradient - grad_old;  // 梯度变化
                
                double sy = y.dot(s);
                if (sy > 1e-10) {  // 避免除零
                    double rho = 1.0 / sy;
                    Matrix2d I = Matrix2d::Identity();
                    H = (I - rho * s * y.transpose()) * H * (I - rho * y * s.transpose()) + rho * s * s.transpose();
                }
            }
            
            // 保存当前值供下次迭代使用
            q_old = q;
            grad_old = gradient;
            error_old = error;
            
            // 更新状态
            Vector2d delta_q = -H * gradient;
            
            // Line search (simple backtracking)
            double alpha_step = 10.0 * pow(0.6, iter); //用1不能收敛
            double c = 0.5;
            double rho_line = 0.5;
            
            Vector2d q_new = q + alpha_step * delta_q;
            C1 = rotatePoint(C1_local_, q_new(0), q_new(1));
            C2 = rotatePoint(C2_local_, q_new(0), q_new(1));
            err1 = (B1 - C1).norm() - l_rod1_;
            err2 = (B2 - C2).norm() - l_rod2_;
            double error_new = err1 * err1 + err2 * err2;
            
            while (error_new > error + c * alpha_step * gradient.dot(delta_q) && alpha_step > 1e-6) {
                alpha_step *= rho_line;
                q_new = q + alpha_step * delta_q;
                C1 = rotatePoint(C1_local_, q_new(0), q_new(1));
                C2 = rotatePoint(C2_local_, q_new(0), q_new(1));
                err1 = (B1 - C1).norm() - l_rod1_;
                err2 = (B2 - C2).norm() - l_rod2_;
                error_new = err1 * err1 + err2 * err2;
            }
            
            // 更新
            q = q_new;
            error = error_new;
        }
        return {q(0), q(1)};
    }



    FastWaistSolver::FastWaistSolver(double r_l) {
        // The r_l parameter might be used for some configuration
        // Currently not used as the mechanisms are symmetric
        (void)r_l; // Suppress unused parameter warning
    }
    
    std::tuple<double, double, double, double, double, double> 
    FastWaistSolver::motorToJointW(double t1, double t2, double v1, double v2, 
                     double tq1, double tq2) const {
        // t1, t2 are motor positions (alpha1, alpha2)
        // Convert from motor to joint angles (position forward kinematics)

        // double roll_guess = -t1 - t2;
        // roll_guess = roll_guess * 0.95;
        double roll_guess = last_waist_roll;
        double pitch_guess = last_waist_pitch;
        
        if (t1 >= 0){
            t1 -= 3.14159;
        }
        else {
            t1 += 3.14159;
        }
        // printf("guess = %f\n", roll_guess);
        auto [roll, pitch] = waist_mechanism_.forwardKinematicsPosition(t1, t2, Vector2d(roll_guess, pitch_guess));

        last_waist_roll = roll;
        last_waist_pitch = pitch;

        // Velocity forward kinematics
        auto [vr, vp] = waist_mechanism_.forwardKinematicsVelocity(roll, pitch, v1, v2);

        // Torque forward kinematics
        auto [tq_r, tq_p] = waist_mechanism_.forwardKinematicsTorque(roll, pitch, tq1, tq2);
        // double vp,vr,tq_p,tq_r = 0.0;s
        return std::make_tuple(roll, pitch, vr, vp, tq_r, tq_p);
    }
    
    std::tuple<double, double, double, double, double, double> 
    FastWaistSolver::jointToMotorW(double tr, double tp, double vr, double vp,
                     double tq_r, double tq_p) const {

        double alpha1=0.0, alpha2=0.0, v1=0.0, v2=0.0;
        auto [tq1, tq2] = waist_mechanism_.inverseKinematicsTorque(tr, tp, tq_r, tq_p);
        
        return std::make_tuple(alpha1, alpha2, v1, v2, tq1, tq2);
    }
    
    std::vector<JointData> FastWaistSolver::motorToJointAll(const std::vector<JointData>& joint_data) const {
        if (joint_data.size() < 12) {
            throw std::runtime_error("joint_data size must be at least 12");
        }
        
        std::vector<JointData> result = joint_data;
        
        // Left side (indices 4 and 5 - motors 5 and 6)
        double tp, tr, vp, vr, tq_p, tq_r;
        std::tie(tr, tp, vr, vp, tq_r, tq_p) = motorToJointW(
            joint_data[13].pos_, joint_data[14].pos_,
            joint_data[13].vel_, joint_data[14].vel_,
            joint_data[13].tau_, joint_data[14].tau_
        );
        
        result[13].pos_ = tp;
        result[13].vel_ = vp;
        result[13].tau_ = tq_p;
        
        result[14].pos_ = tr;
        result[14].vel_ = vr;
        result[14].tau_ = tq_r;
        
        return result;
    }
    
    void FastWaistSolver::jointToMotorAll(std::vector<double>& q_d, std::vector<double>& v_d,
                        std::vector<double>& tq_d) const {
        
        // Left side (indices 4 and 5)
        double alpha1, alpha2, v1, v2, tq1, tq2;
        std::tie(alpha1, alpha2, v1, v2, tq1, tq2) = jointToMotorW(
            q_d[14], q_d[13], v_d[14], v_d[13], tq_d[14], tq_d[13]
        );
        
        q_d[13] = alpha1;
        q_d[14] = alpha2;
        v_d[13] = v1;
        v_d[14] = v2;
        tq_d[13] = tq1;
        tq_d[14] = tq2;
    }

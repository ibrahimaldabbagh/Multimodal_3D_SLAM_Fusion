// Enhanced ESKF with Constant Acceleration Model
// Copyright (c) 2025
// License: MIT

#ifndef FUSION_ESKF_CA_HPP
#define FUSION_ESKF_CA_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

namespace fusion {

/**
 * @brief Extended State Kalman Filter with Constant Acceleration Model
 * 
 * State vector (24-dimensional):
 * [0:3]   position (x, y, z)
 * [3:6]   velocity (vx, vy, vz)
 * [6:9]   acceleration (ax, ay, az)
 * [9:12]  orientation error (theta_x, theta_y, theta_z) - small angle
 * [12:15] gyro bias (bgx, bgy, bgz)
 * [15:18] accel bias (bax, bay, baz)
 * [18:21] angular velocity (wx, wy, wz)
 * [21:24] angular acceleration (alpha_x, alpha_y, alpha_z)
 */

struct EskfCAParams {
  // Process noise
  double sigma_g{1.5e-3};      // gyro noise (rad/s/√Hz)
  double sigma_a{3.0e-2};      // accel noise (m/s²/√Hz)
  double sigma_bg{1.0e-5};     // gyro bias random walk
  double sigma_ba{5.0e-5};     // accel bias random walk
  double sigma_v{1.0e-3};      // velocity random walk
  double sigma_acc{5.0e-3};    // acceleration random walk
  double sigma_w{1.0e-4};      // angular velocity noise
  double sigma_alpha{1.0e-3};  // angular acceleration noise
  
  // Initial covariances
  double init_pos_var{1.0};
  double init_vel_var{0.1};
  double init_acc_var{0.01};
  double init_att_var{0.01};
  double init_bg_var{1e-4};
  double init_ba_var{1e-3};
  double init_w_var{0.01};
  double init_alpha_var{0.001};
  
  Eigen::Vector3d gravity{0.0, 0.0, -9.81};
};

class ESKF_CA {
public:
  static constexpr int STATE_DIM = 24;
  
  ESKF_CA() {
    reset();
  }
  
  void setParams(const EskfCAParams& p) {
    params_ = p;
    initializeCovariance();
  }
  
  void reset() {
    // Nominal state
    p_.setZero();
    v_.setZero();
    a_.setZero();
    q_ = Eigen::Quaterniond::Identity();
    bg_.setZero();
    ba_.setZero();
    w_.setZero();
    alpha_.setZero();
    
    // Error state covariance
    P_.setIdentity();
    P_ *= 1.0;
    
    initializeCovariance();
  }
  
  /**
   * @brief Propagate state with IMU measurements
   * @param dt Time step
   * @param gyro Angular velocity measurement
   * @param accel Linear acceleration measurement
   */
  void propagate(double dt, const Eigen::Vector3d& gyro, const Eigen::Vector3d& accel) {
    if (dt <= 0.0 || dt > 1.0) return;
    
    // Corrected measurements
    Eigen::Vector3d w_corrected = gyro - bg_;
    Eigen::Vector3d a_corrected = accel - ba_;
    
    // Current rotation matrix
    Eigen::Matrix3d R = q_.toRotationMatrix();
    
    // === Nominal state propagation with CA model ===
    
    // Update angular acceleration (first-order hold)
    Eigen::Vector3d alpha_new = (w_corrected - w_) / dt;
    alpha_ = 0.7 * alpha_ + 0.3 * alpha_new; // Smoothing
    
    // Update angular velocity
    w_ = w_corrected;
    
    // Update orientation (quaternion integration)
    Eigen::Vector3d w_avg = w_ + 0.5 * alpha_ * dt;
    double w_norm = w_avg.norm();
    if (w_norm > 1e-8) {
      double half_theta = 0.5 * w_norm * dt;
      Eigen::Quaterniond dq;
      dq.w() = std::cos(half_theta);
      dq.vec() = (std::sin(half_theta) / w_norm) * w_avg;
      q_ = q_ * dq;
      q_.normalize();
    }
    
    // Update acceleration in world frame
    Eigen::Vector3d a_world = R * a_corrected + params_.gravity;
    Eigen::Vector3d a_new = (v_ - v_) / dt; // Observed from velocity change
    a_ = 0.6 * a_ + 0.4 * a_world; // Blend with direct measurement
    
    // Update velocity (with jerk consideration)
    v_ += a_ * dt + 0.5 * (a_world - a_) * dt;
    
    // Update position
    p_ += v_ * dt + 0.5 * a_ * dt * dt;
    
    // === Error state propagation ===
    
    // State transition matrix F (24x24)
    Eigen::MatrixXd F = Eigen::MatrixXd::Zero(STATE_DIM, STATE_DIM);
    
    // Position derivatives
    F.block<3,3>(0,3) = Eigen::Matrix3d::Identity(); // dp/dv
    F.block<3,3>(0,6) = 0.5 * dt * dt * Eigen::Matrix3d::Identity(); // dp/da
    
    // Velocity derivatives
    F.block<3,3>(3,6) = Eigen::Matrix3d::Identity(); // dv/da
    F.block<3,3>(3,9) = -R * skew(a_corrected); // dv/dtheta
    F.block<3,3>(3,15) = -R; // dv/dba
    
    // Acceleration derivatives (simplified)
    F.block<3,3>(6,9) = -R * skew(a_corrected) / dt; // da/dtheta
    F.block<3,3>(6,15) = -R / dt; // da/dba
    
    // Orientation error derivatives
    F.block<3,3>(9,9) = -skew(w_corrected); // dtheta/dtheta
    F.block<3,3>(9,12) = -Eigen::Matrix3d::Identity(); // dtheta/dbg
    F.block<3,3>(9,18) = -Eigen::Matrix3d::Identity() * dt; // dtheta/dw
    
    // Gyro bias (random walk)
    // dbg/dbg = 0 (already zero in F)
    
    // Accel bias (random walk)
    // dba/dba = 0
    
    // Angular velocity derivatives
    F.block<3,3>(18,12) = -Eigen::Matrix3d::Identity() / dt; // dw/dbg
    F.block<3,3>(18,21) = Eigen::Matrix3d::Identity(); // dw/dalpha
    
    // Angular acceleration (random walk with decay)
    F.block<3,3>(21,21) = -0.1 * Eigen::Matrix3d::Identity(); // Small damping
    
    // Discrete state transition
    Eigen::MatrixXd Phi = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM) + F * dt;
    
    // Process noise matrix Q
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(STATE_DIM, STATE_DIM);
    
    // Position noise (from velocity uncertainty)
    Q.block<3,3>(0,0) = params_.sigma_v * params_.sigma_v * dt * dt * Eigen::Matrix3d::Identity();
    
    // Velocity noise (from acceleration uncertainty)
    Q.block<3,3>(3,3) = params_.sigma_a * params_.sigma_a * dt * Eigen::Matrix3d::Identity();
    
    // Acceleration noise
    Q.block<3,3>(6,6) = params_.sigma_acc * params_.sigma_acc * dt * Eigen::Matrix3d::Identity();
    
    // Orientation noise (from gyro)
    Q.block<3,3>(9,9) = params_.sigma_g * params_.sigma_g * dt * Eigen::Matrix3d::Identity();
    
    // Gyro bias noise
    Q.block<3,3>(12,12) = params_.sigma_bg * params_.sigma_bg * dt * Eigen::Matrix3d::Identity();
    
    // Accel bias noise
    Q.block<3,3>(15,15) = params_.sigma_ba * params_.sigma_ba * dt * Eigen::Matrix3d::Identity();
    
    // Angular velocity noise
    Q.block<3,3>(18,18) = params_.sigma_w * params_.sigma_w * dt * Eigen::Matrix3d::Identity();
    
    // Angular acceleration noise
    Q.block<3,3>(21,21) = params_.sigma_alpha * params_.sigma_alpha * dt * Eigen::Matrix3d::Identity();
    
    // Covariance propagation
    P_ = Phi * P_ * Phi.transpose() + Q;
    
    // Ensure symmetry and positive definiteness
    P_ = 0.5 * (P_ + P_.transpose());
    
    // Add small diagonal for numerical stability
    P_ += 1e-9 * Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);
  }
  
  /**
   * @brief Correct position measurement
   */
  void correctPosition(const Eigen::Vector3d& z_pos, const Eigen::Matrix3d& R_pos) {
    // Measurement model: z = p + noise
    Eigen::Vector3d innov = z_pos - p_;
    
    // Measurement matrix (3x24)
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, STATE_DIM);
    H.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
    
    updateState(innov, H, R_pos);
  }
  
  /**
   * @brief Correct velocity measurement
   */
  void correctVelocity(const Eigen::Vector3d& z_vel, const Eigen::Matrix3d& R_vel) {
    Eigen::Vector3d innov = z_vel - v_;
    
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, STATE_DIM);
    H.block<3,3>(0,3) = Eigen::Matrix3d::Identity();
    
    updateState(innov, H, R_vel);
  }
  
  /**
   * @brief Correct position and yaw (from visual-inertial)
   */
  void correctPoseWithYaw(const Eigen::Vector3d& z_pos, double z_yaw,
                          const Eigen::Matrix3d& R_pos, double R_yaw) {
    // First correct position
    correctPosition(z_pos, R_pos);
    
    // Then correct yaw
    double yaw_current = quaternionToYaw(q_);
    double yaw_innov = normalizeAngle(z_yaw - yaw_current);
    
    // Measurement matrix for yaw
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1, STATE_DIM);
    // Yaw is primarily affected by theta_z (error in z-rotation)
    H(0, 11) = 1.0; // theta_z component
    
    Eigen::Matrix<double,1,1> R_yaw_mat;
    R_yaw_mat(0,0) = R_yaw;
    
    Eigen::VectorXd innov(1);
    innov(0) = yaw_innov;
    
    updateState(innov, H, R_yaw_mat);
  }
  
  /**
   * @brief Correct altitude only
   */
  void correctAltitude(double z_alt, double R_alt) {
    double innov = z_alt - p_(2);
    
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1, STATE_DIM);
    H(0, 2) = 1.0;
    
    Eigen::Matrix<double,1,1> R_mat;
    R_mat(0,0) = R_alt;
    
    Eigen::VectorXd innov_vec(1);
    innov_vec(0) = innov;
    
    updateState(innov_vec, H, R_mat);
  }
  
  /**
   * @brief Correct along-track distance (speed-integrated distance)
   */
  void correctAlongTrack(double z_dist, double R_dist) {
    // Project velocity onto current heading
    Eigen::Vector3d heading = q_ * Eigen::Vector3d::UnitX();
    double v_along = v_.dot(heading);
    
    static double integrated_dist = 0.0;
    static double last_time = 0.0;
    double dt = 0.1; // Approximate
    integrated_dist += v_along * dt;
    
    double innov = z_dist - integrated_dist;
    
    // Measurement matrix
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1, STATE_DIM);
    // Partial derivative of integrated distance w.r.t. velocity
    H.block<1,3>(0,3) = heading.transpose() * dt;
    
    Eigen::Matrix<double,1,1> R_mat;
    R_mat(0,0) = R_dist;
    
    Eigen::VectorXd innov_vec(1);
    innov_vec(0) = innov;
    
    updateState(innov_vec, H, R_mat);
  }
  
  /**
   * @brief Full 6-DOF pose correction
   */
  void correctFullPose(const Eigen::Vector3d& z_pos, const Eigen::Quaterniond& z_quat,
                       const Eigen::Matrix<double,6,6>& R_pose) {
    // Position innovation
    Eigen::Vector3d pos_innov = z_pos - p_;
    
    // Orientation innovation (quaternion error to rotation vector)
    Eigen::Quaterniond q_error = z_quat * q_.conjugate();
    Eigen::Vector3d att_innov = 2.0 * q_error.vec();
    if (q_error.w() < 0) att_innov = -att_innov;
    
    // Combined innovation
    Eigen::VectorXd innov(6);
    innov.head<3>() = pos_innov;
    innov.tail<3>() = att_innov;
    
    // Measurement matrix
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(6, STATE_DIM);
    H.block<3,3>(0,0) = Eigen::Matrix3d::Identity(); // Position
    H.block<3,3>(3,9) = Eigen::Matrix3d::Identity(); // Orientation error
    
    updateState(innov, H, R_pose);
  }
  
  // Getters
  const Eigen::Vector3d& p() const { return p_; }
  const Eigen::Vector3d& v() const { return v_; }
  const Eigen::Vector3d& a() const { return a_; }
  const Eigen::Quaterniond& q() const { return q_; }
  const Eigen::Vector3d& bg() const { return bg_; }
  const Eigen::Vector3d& ba() const { return ba_; }
  const Eigen::Vector3d& w() const { return w_; }
  const Eigen::Vector3d& alpha() const { return alpha_; }
  const Eigen::MatrixXd& P() const { return P_; }
  
  // Setters for initialization
  void setPosition(const Eigen::Vector3d& pos) { p_ = pos; }
  void setVelocity(const Eigen::Vector3d& vel) { v_ = vel; }
  void setOrientation(const Eigen::Quaterniond& quat) { q_ = quat; q_.normalize(); }
  
private:
  void initializeCovariance() {
    P_.setZero();
    P_.block<3,3>(0,0) = params_.init_pos_var * Eigen::Matrix3d::Identity();
    P_.block<3,3>(3,3) = params_.init_vel_var * Eigen::Matrix3d::Identity();
    P_.block<3,3>(6,6) = params_.init_acc_var * Eigen::Matrix3d::Identity();
    P_.block<3,3>(9,9) = params_.init_att_var * Eigen::Matrix3d::Identity();
    P_.block<3,3>(12,12) = params_.init_bg_var * Eigen::Matrix3d::Identity();
    P_.block<3,3>(15,15) = params_.init_ba_var * Eigen::Matrix3d::Identity();
    P_.block<3,3>(18,18) = params_.init_w_var * Eigen::Matrix3d::Identity();
    P_.block<3,3>(21,21) = params_.init_alpha_var * Eigen::Matrix3d::Identity();
  }
  
  template<typename Derived1, typename Derived2, typename Derived3>
  void updateState(const Eigen::MatrixBase<Derived1>& innov,
                   const Eigen::MatrixBase<Derived2>& H,
                   const Eigen::MatrixBase<Derived3>& R) {
    // Kalman gain
    auto S = H * P_ * H.transpose() + R;
    auto K = P_ * H.transpose() * S.inverse();
    
    // Error state update
    Eigen::VectorXd dx = K * innov;
    
    // Inject error state into nominal state
    p_ += dx.segment<3>(0);
    v_ += dx.segment<3>(3);
    a_ += dx.segment<3>(6);
    
    // Orientation update
    Eigen::Vector3d dtheta = dx.segment<3>(9);
    if (dtheta.norm() > 1e-8) {
      Eigen::Quaterniond dq = expQuaternion(dtheta);
      q_ = q_ * dq;
      q_.normalize();
    }
    
    bg_ += dx.segment<3>(12);
    ba_ += dx.segment<3>(15);
    w_ += dx.segment<3>(18);
    alpha_ += dx.segment<3>(21);
    
    // Covariance update (Joseph form for numerical stability)
    Eigen::MatrixXd I_KH = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM) - K * H;
    P_ = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();
    
    // Ensure symmetry
    P_ = 0.5 * (P_ + P_.transpose());
  }
  
  // Helper: skew-symmetric matrix
  Eigen::Matrix3d skew(const Eigen::Vector3d& v) const {
    Eigen::Matrix3d m;
    m <<    0, -v.z(),  v.y(),
         v.z(),     0, -v.x(),
        -v.y(),  v.x(),     0;
    return m;
  }
  
  // Helper: small angle to quaternion
  Eigen::Quaterniond expQuaternion(const Eigen::Vector3d& omega) const {
    double theta = omega.norm();
    if (theta < 1e-8) {
      return Eigen::Quaterniond(1.0, 0.5*omega.x(), 0.5*omega.y(), 0.5*omega.z());
    }
    double s = std::sin(0.5 * theta) / theta;
    return Eigen::Quaterniond(std::cos(0.5*theta), s*omega.x(), s*omega.y(), s*omega.z());
  }
  
  // Helper: extract yaw from quaternion
  double quaternionToYaw(const Eigen::Quaterniond& q) const {
    return std::atan2(2.0*(q.w()*q.z() + q.x()*q.y()),
                     1.0 - 2.0*(q.y()*q.y() + q.z()*q.z()));
  }
  
  // Helper: normalize angle to [-pi, pi]
  double normalizeAngle(double angle) const {
    while (angle > M_PI) angle -= 2.0*M_PI;
    while (angle < -M_PI) angle += 2.0*M_PI;
    return angle;
  }
  
  // State variables (nominal state)
  Eigen::Vector3d p_;      // Position
  Eigen::Vector3d v_;      // Velocity
  Eigen::Vector3d a_;      // Acceleration
  Eigen::Quaterniond q_;   // Orientation
  Eigen::Vector3d bg_;     // Gyro bias
  Eigen::Vector3d ba_;     // Accel bias
  Eigen::Vector3d w_;      // Angular velocity
  Eigen::Vector3d alpha_;  // Angular acceleration
  
  // Error state covariance
  Eigen::MatrixXd P_;
  
  // Parameters
  EskfCAParams params_;
};

} // namespace fusion

#endif // FUSION_ESKF_CA_HPP

// mvSLAM stereo+IMU implementation
// Copyright (c) 2025
// License: MIT
// Clean-room code. No external proprietary content.

#pragma once
/**
 * @file filters.hpp
 * @brief Small utility filters used to stabilize noisy estimates.
 */

#include <algorithm>
#include <cmath>

namespace mvslam {

/**
 * @brief Exponential Moving Average (EMA) for scalar signals.
 * y_k = alpha * x_k + (1 - alpha) * y_{k-1}
 * alpha in (0,1]; larger alpha -> faster response, smaller -> smoother.
 */
class EMA {
public:
  explicit EMA(double alpha=0.2) : alpha_(std::clamp(alpha, 1e-6, 1.0)), initialized_(false), y_(0.0) {}
  void reset() { initialized_ = false; y_ = 0.0; }
  double update(double x) {
    if(!initialized_) { y_ = x; initialized_ = true; return y_; }
    y_ = alpha_ * x + (1.0 - alpha_) * y_;
    return y_;
  }
  double value() const { return y_; }
private:
  double alpha_;
  bool initialized_;
  double y_;
};

/**
 * @brief Minimal 1D Kalman filter for constant-velocity model.
 * State: [position; velocity], Measurement: position
 * Useful as a gentle stabilizer for per-axis translation.
 */
class Kalman1D {
public:
  Kalman1D(double q_pos=1e-3, double q_vel=1e-2, double r_meas=1e-2)
  : x_{0.0, 0.0} {
    // State covariance
    P_[0][0]=1.0; P_[0][1]=0.0;
    P_[1][0]=0.0; P_[1][1]=1.0;
    // Process noise
    Q_[0][0]=q_pos; Q_[0][1]=0.0;
    Q_[1][0]=0.0;   Q_[1][1]=q_vel;
    // Measurement noise
    R_=r_meas;
  }

  void reset(double pos=0.0, double vel=0.0){
    x_[0]=pos; x_[1]=vel;
    P_[0][0]=1.0; P_[0][1]=0.0;
    P_[1][0]=0.0; P_[1][1]=1.0;
  }

  void predict(double dt){
    // x = F x, where F = [[1, dt],[0,1]]
    double x0 = x_[0] + dt * x_[1];
    double x1 = x_[1];
    x_[0]=x0; x_[1]=x1;

    // P = F P F^T + Q
    double p00 = P_[0][0] + dt*(P_[1][0]+P_[0][1]) + dt*dt*P_[1][1] + Q_[0][0];
    double p01 = P_[0][1] + dt*P_[1][1] + Q_[0][1];
    double p10 = P_[1][0] + dt*P_[1][1] + Q_[1][0];
    double p11 = P_[1][1] + Q_[1][1];
    P_[0][0]=p00; P_[0][1]=p01;
    P_[1][0]=p10; P_[1][1]=p11;
  }

  void update(double z){
    // Measurement matrix H = [1, 0]
    double y = z - x_[0];                        // innovation
    double S = P_[0][0] + R_;                    // innovation covariance
    double K0 = P_[0][0] / S;                    // Kalman gain
    double K1 = P_[1][0] / S;

    x_[0] += K0 * y;
    x_[1] += K1 * y;

    double p00 = (1.0 - K0) * P_[0][0];
    double p01 = (1.0 - K0) * P_[0][1];
    double p10 = P_[1][0] - K1 * P_[0][0];
    double p11 = P_[1][1] - K1 * P_[0][1];
    P_[0][0]=p00; P_[0][1]=p01;
    P_[1][0]=p10; P_[1][1]=p11;
  }

  double position() const { return x_[0]; }
  double velocity() const { return x_[1]; }
private:
  double x_[2];
  double P_[2][2];
  double Q_[2][2];
  double R_;
};

} // namespace mvslam

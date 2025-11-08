// Sensor Models and Coordinate Transformations
// Copyright (c) 2025
// License: MIT

#ifndef FUSION_SENSOR_MODELS_HPP
#define FUSION_SENSOR_MODELS_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

namespace fusion {

/**
 * @brief Geographic coordinate transformations
 */
class GeoCoordinates {
public:
  // WGS84 ellipsoid constants
  static constexpr double WGS84_A = 6378137.0;           // Semi-major axis (m)
  static constexpr double WGS84_B = 6356752.314245;      // Semi-minor axis (m)
  static constexpr double WGS84_E2 = 0.00669437999014;   // First eccentricity squared
  
  /**
   * @brief Convert LLA to ECEF coordinates
   * @param lat Latitude (degrees)
   * @param lon Longitude (degrees)
   * @param alt Altitude (m)
   * @return ECEF coordinates (m)
   */
  static Eigen::Vector3d llaToEcef(double lat, double lon, double alt) {
    double lat_rad = lat * M_PI / 180.0;
    double lon_rad = lon * M_PI / 180.0;
    
    double sin_lat = std::sin(lat_rad);
    double cos_lat = std::cos(lat_rad);
    double sin_lon = std::sin(lon_rad);
    double cos_lon = std::cos(lon_rad);
    
    double N = WGS84_A / std::sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat);
    
    Eigen::Vector3d ecef;
    ecef.x() = (N + alt) * cos_lat * cos_lon;
    ecef.y() = (N + alt) * cos_lat * sin_lon;
    ecef.z() = (N * (1.0 - WGS84_E2) + alt) * sin_lat;
    
    return ecef;
  }
  
  /**
   * @brief Convert ECEF to LLA coordinates
   * @param ecef ECEF coordinates (m)
   * @return [latitude (deg), longitude (deg), altitude (m)]
   */
  static Eigen::Vector3d ecefToLla(const Eigen::Vector3d& ecef) {
    double x = ecef.x();
    double y = ecef.y();
    double z = ecef.z();
    
    double lon = std::atan2(y, x);
    
    double p = std::sqrt(x*x + y*y);
    double lat = std::atan2(z, p * (1.0 - WGS84_E2));
    
    // Iterative refinement
    for (int i = 0; i < 5; ++i) {
      double sin_lat = std::sin(lat);
      double N = WGS84_A / std::sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat);
      double alt = p / std::cos(lat) - N;
      lat = std::atan2(z, p * (1.0 - WGS84_E2 * N / (N + alt)));
    }
    
    double sin_lat = std::sin(lat);
    double N = WGS84_A / std::sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat);
    double alt = p / std::cos(lat) - N;
    
    return Eigen::Vector3d(lat * 180.0 / M_PI, lon * 180.0 / M_PI, alt);
  }
  
  /**
   * @brief Convert LLA to local ENU (East-North-Up) coordinates
   * @param lat Latitude (degrees)
   * @param lon Longitude (degrees)
   * @param alt Altitude (m)
   * @param lat_ref Reference latitude (degrees)
   * @param lon_ref Reference longitude (degrees)
   * @param alt_ref Reference altitude (m)
   * @return ENU coordinates (m)
   */
  static Eigen::Vector3d llaToEnu(double lat, double lon, double alt,
                                  double lat_ref, double lon_ref, double alt_ref) {
    Eigen::Vector3d ecef = llaToEcef(lat, lon, alt);
    Eigen::Vector3d ecef_ref = llaToEcef(lat_ref, lon_ref, alt_ref);
    
    Eigen::Vector3d ecef_delta = ecef - ecef_ref;
    
    double lat_rad = lat_ref * M_PI / 180.0;
    double lon_rad = lon_ref * M_PI / 180.0;
    
    double sin_lat = std::sin(lat_rad);
    double cos_lat = std::cos(lat_rad);
    double sin_lon = std::sin(lon_rad);
    double cos_lon = std::cos(lon_rad);
    
    Eigen::Matrix3d R_ecef_to_enu;
    R_ecef_to_enu << -sin_lon,           cos_lon,          0,
                     -sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat,
                      cos_lat * cos_lon,  cos_lat * sin_lon, sin_lat;
    
    return R_ecef_to_enu * ecef_delta;
  }
  
  /**
   * @brief Flat-earth approximation for small areas
   * @param lat Latitude (degrees)
   * @param lon Longitude (degrees)
   * @param alt Altitude (m)
   * @param lat_ref Reference latitude (degrees)
   * @param lon_ref Reference longitude (degrees)
   * @param alt_ref Reference altitude (m)
   * @return ENU coordinates (m)
   */
  static Eigen::Vector3d llaToEnuFlat(double lat, double lon, double alt,
                                      double lat_ref, double lon_ref, double alt_ref) {
    double dlat = (lat - lat_ref) * M_PI / 180.0;
    double dlon = (lon - lon_ref) * M_PI / 180.0;
    double dalt = alt - alt_ref;
    
    double lat_avg = (lat + lat_ref) * 0.5 * M_PI / 180.0;
    
    double east = WGS84_A * dlon * std::cos(lat_avg);
    double north = WGS84_A * dlat;
    double up = dalt;
    
    return Eigen::Vector3d(east, north, up);
  }
};

/**
 * @brief IMU sensor model with calibration
 */
class ImuModel {
public:
  struct Calibration {
    // Misalignment matrices
    Eigen::Matrix3d T_gyro = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d T_accel = Eigen::Matrix3d::Identity();
    
    // Scale factors
    Eigen::Vector3d scale_gyro = Eigen::Vector3d::Ones();
    Eigen::Vector3d scale_accel = Eigen::Vector3d::Ones();
    
    // Biases
    Eigen::Vector3d bias_gyro = Eigen::Vector3d::Zero();
    Eigen::Vector3d bias_accel = Eigen::Vector3d::Zero();
    
    // Gravity in sensor frame (for static calibration)
    Eigen::Vector3d gravity_sensor = Eigen::Vector3d(0, 0, 9.81);
  };
  
  /**
   * @brief Apply calibration to raw measurements
   */
  static void calibrate(const Eigen::Vector3d& gyro_raw,
                       const Eigen::Vector3d& accel_raw,
                       const Calibration& calib,
                       Eigen::Vector3d& gyro_cal,
                       Eigen::Vector3d& accel_cal) {
    // Remove bias
    Eigen::Vector3d gyro_unbiased = gyro_raw - calib.bias_gyro;
    Eigen::Vector3d accel_unbiased = accel_raw - calib.bias_accel;
    
    // Apply scale factors
    gyro_unbiased = gyro_unbiased.cwiseProduct(calib.scale_gyro);
    accel_unbiased = accel_unbiased.cwiseProduct(calib.scale_accel);
    
    // Apply misalignment correction
    gyro_cal = calib.T_gyro * gyro_unbiased;
    accel_cal = calib.T_accel * accel_unbiased;
  }
  
  /**
   * @brief Estimate static bias from measurements
   */
  static void estimateStaticBias(const std::vector<Eigen::Vector3d>& gyro_samples,
                                 const std::vector<Eigen::Vector3d>& accel_samples,
                                 Calibration& calib) {
    if (gyro_samples.empty() || accel_samples.empty()) return;
    
    // Compute mean
    Eigen::Vector3d gyro_mean = Eigen::Vector3d::Zero();
    Eigen::Vector3d accel_mean = Eigen::Vector3d::Zero();
    
    for (const auto& g : gyro_samples) gyro_mean += g;
    for (const auto& a : accel_samples) accel_mean += a;
    
    gyro_mean /= gyro_samples.size();
    accel_mean /= accel_samples.size();
    
    // Gyro bias is the mean (should be zero when static)
    calib.bias_gyro = gyro_mean;
    
    // Accel bias is mean minus expected gravity
    calib.bias_accel = accel_mean - calib.gravity_sensor;
  }
};

/**
 * @brief Barometer sensor model
 */
class BarometerModel {
public:
  /**
   * @brief Convert pressure to altitude (ISA model)
   * @param pressure Pressure (Pa)
   * @param pressure_ref Reference pressure at sea level (Pa)
   * @return Altitude (m)
   */
  static double pressureToAltitude(double pressure, double pressure_ref = 101325.0) {
    constexpr double T0 = 288.15;  // Sea level temperature (K)
    constexpr double L = 0.0065;   // Temperature lapse rate (K/m)
    constexpr double g = 9.80665;  // Gravity (m/s²)
    constexpr double R = 287.05;   // Gas constant (J/(kg·K))
    
    double exponent = R * L / g;
    double altitude = (T0 / L) * (1.0 - std::pow(pressure / pressure_ref, exponent));
    
    return altitude;
  }
  
  /**
   * @brief Convert altitude to pressure
   */
  static double altitudeToPressure(double altitude, double pressure_ref = 101325.0) {
    constexpr double T0 = 288.15;
    constexpr double L = 0.0065;
    constexpr double g = 9.80665;
    constexpr double R = 287.05;
    
    double exponent = g / (R * L);
    double pressure = pressure_ref * std::pow(1.0 - (L * altitude) / T0, exponent);
    
    return pressure;
  }
};

/**
 * @brief Magnetometer sensor model
 */
class MagnetometerModel {
public:
  /**
   * @brief Compute magnetic declination
   * @param lat Latitude (degrees)
   * @param lon Longitude (degrees)
   * @return Declination (degrees, positive East)
   */
  static double computeDeclination(double lat, double lon) {
    // Simplified WMM model (approximate)
    // For accurate results, use full WMM
    double dec = -0.0573 * lon - 0.0105 * lat + 5.76;
    return dec;
  }
  
  /**
   * @brief Correct magnetic heading to true heading
   */
  static double magneticToTrue(double mag_heading, double declination) {
    return mag_heading + declination;
  }
  
  /**
   * @brief Extract yaw from magnetometer
   */
  static double computeYaw(const Eigen::Vector3d& mag,
                          const Eigen::Quaterniond& q_imu_to_world) {
    // Rotate magnetometer to world frame
    Eigen::Matrix3d R = q_imu_to_world.toRotationMatrix();
    Eigen::Vector3d mag_world = R * mag;
    
    // Compute yaw (angle from magnetic north)
    double yaw = std::atan2(mag_world.y(), mag_world.x());
    
    return yaw;
  }
};

/**
 * @brief Utility functions for quaternion operations
 */
class QuaternionUtils {
public:
  /**
   * @brief Convert rotation vector to quaternion (exponential map)
   */
  static Eigen::Quaterniond expMap(const Eigen::Vector3d& omega) {
    double theta = omega.norm();
    
    if (theta < 1e-10) {
      // Small angle approximation
      return Eigen::Quaterniond(1.0, 0.5 * omega.x(), 0.5 * omega.y(), 0.5 * omega.z());
    }
    
    double half_theta = 0.5 * theta;
    double s = std::sin(half_theta) / theta;
    
    return Eigen::Quaterniond(std::cos(half_theta), s * omega.x(), s * omega.y(), s * omega.z());
  }
  
  /**
   * @brief Convert quaternion to rotation vector (logarithmic map)
   */
  static Eigen::Vector3d logMap(const Eigen::Quaterniond& q) {
    Eigen::Vector3d vec = q.vec();
    double vec_norm = vec.norm();
    
    if (vec_norm < 1e-10) {
      return 2.0 * vec;
    }
    
    double theta = 2.0 * std::atan2(vec_norm, q.w());
    return (theta / vec_norm) * vec;
  }
  
  /**
   * @brief Quaternion difference (q2 - q1 in tangent space)
   */
  static Eigen::Vector3d quaternionDifference(const Eigen::Quaterniond& q1,
                                              const Eigen::Quaterniond& q2) {
    Eigen::Quaterniond q_error = q2 * q1.conjugate();
    return logMap(q_error);
  }
  
  /**
   * @brief Extract roll, pitch, yaw from quaternion
   */
  static Eigen::Vector3d toRPY(const Eigen::Quaterniond& q) {
    Eigen::Vector3d rpy;
    
    // Roll (x-axis rotation)
    double sinr_cosp = 2.0 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
    rpy.x() = std::atan2(sinr_cosp, cosr_cosp);
    
    // Pitch (y-axis rotation)
    double sinp = 2.0 * (q.w() * q.y() - q.z() * q.x());
    if (std::abs(sinp) >= 1.0) {
      rpy.y() = std::copysign(M_PI / 2.0, sinp);
    } else {
      rpy.y() = std::asin(sinp);
    }
    
    // Yaw (z-axis rotation)
    double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    rpy.z() = std::atan2(siny_cosp, cosy_cosp);
    
    return rpy;
  }
  
  /**
   * @brief Create quaternion from roll, pitch, yaw
   */
  static Eigen::Quaterniond fromRPY(double roll, double pitch, double yaw) {
    double cy = std::cos(yaw * 0.5);
    double sy = std::sin(yaw * 0.5);
    double cp = std::cos(pitch * 0.5);
    double sp = std::sin(pitch * 0.5);
    double cr = std::cos(roll * 0.5);
    double sr = std::sin(roll * 0.5);
    
    Eigen::Quaterniond q;
    q.w() = cr * cp * cy + sr * sp * sy;
    q.x() = sr * cp * cy - cr * sp * sy;
    q.y() = cr * sp * cy + sr * cp * sy;
    q.z() = cr * cp * sy - sr * sp * cy;
    
    return q;
  }
  
  /**
   * @brief Spherical linear interpolation (SLERP)
   */
  static Eigen::Quaterniond slerp(const Eigen::Quaterniond& q1,
                                  const Eigen::Quaterniond& q2,
                                  double t) {
    return q1.slerp(t, q2);
  }
};

/**
 * @brief Statistical utilities
 */
class Statistics {
public:
  /**
   * @brief Compute Mahalanobis distance
   */
  static double mahalanobisDistance(const Eigen::VectorXd& x,
                                    const Eigen::VectorXd& mean,
                                    const Eigen::MatrixXd& cov) {
    Eigen::VectorXd diff = x - mean;
    return std::sqrt(diff.transpose() * cov.inverse() * diff);
  }
  
  /**
   * @brief Chi-squared test
   */
  static bool chiSquaredTest(double mahal_dist, int dof, double confidence = 0.95) {
    // Simplified chi-squared critical values
    std::vector<double> critical_values = {3.84, 5.99, 7.81, 9.49, 11.07, 12.59, 14.07};
    
    if (dof <= 0 || dof > 7) return true; // Pass if out of range
    
    return mahal_dist * mahal_dist < critical_values[dof - 1];
  }
  
  /**
   * @brief Compute weighted mean
   */
  static Eigen::VectorXd weightedMean(const std::vector<Eigen::VectorXd>& samples,
                                      const std::vector<double>& weights) {
    if (samples.empty() || samples.size() != weights.size()) {
      return Eigen::VectorXd::Zero(0);
    }
    
    Eigen::VectorXd mean = Eigen::VectorXd::Zero(samples[0].size());
    double weight_sum = 0.0;
    
    for (size_t i = 0; i < samples.size(); ++i) {
      mean += weights[i] * samples[i];
      weight_sum += weights[i];
    }
    
    if (weight_sum > 0) {
      mean /= weight_sum;
    }
    
    return mean;
  }
};

} // namespace fusion

#endif // FUSION_SENSOR_MODELS_HPP

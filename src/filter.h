#ifndef SENSOR_FUSION_FILTER_H
#define SENSOR_FUSION_FILTER_H

#include <Eigen/Geometry>
#include <iostream>
#include <optional>

#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "structures.h"
#include "utils.h"

// nuber of error states

/*
 * Error state Kalman filter implementation based on:
 * Quaternion kinematics for the error-state Kalman filter
 * by Joan Sola -> pdf: https://arxiv.org/pdf/1711.02508.pdf
 * Read for more detail.
 *
 * We use imu acceleration and angular velocity measurements as
 * inputs for the prediction model.
 * For the update model we use pose measurements and correct
 * the predicted state with the errors.
 * For the implementation to work as expected it is expected
 * that imu measurements have at least a few times higher
 * rate than the sensors used for pose measurements
 * (eg. imu: 100hz and sensor: 25 hz )
 *
 * "True State" (266)
 *  [ Position 3x1 , Lin. Velocity 3x1, Quaternion 4x1, Lin. Acceleration bias 3x1,
 *    Angular velocity bias 3x1, gravity 3x1, drift translation 3x1, drift rotation 4x1 ]
 *
 *  Error state
 *
 * [ Position 3x1 , Lin. Velocity 3x1, Angle Axis 3x1, Lin. Acceleration bias 3x1,
 *    Angular velocity bias 3x1, gravity 3x1, drift translation 3x1, drift angle axis 3x1
 * ]
 *
 */

using namespace Eigen;
class EsEkf2
{
private:
  static constexpr auto N_STATES        = 24;
  static constexpr auto N_STATES_2      = N_STATES / 2;
  static constexpr auto POSITION_IDX    = 0;
  static constexpr auto VELOCITY_IDX    = 3;
  static constexpr auto ANGLE_AXIS_IDX  = 6;
  static constexpr auto ACC_BIAS_IDX    = 9;
  static constexpr auto GYRO_BIAS_IDX   = 12;
  static constexpr auto GRAVITY_IDX     = 15;
  static constexpr auto DRIFT_TRANS_IDX = 18;
  static constexpr auto DRIFT_ROT_IDX   = 21;

  Vector3d                           m_est_position;
  Vector3d                           m_est_lin_velocity;
  Vector3d                           m_est_gyro_bias;
  Vector3d                           m_est_acc_bias;
  Vector3d                           m_est_gravity;
  Quaterniond                        m_est_quaternion;
  Matrix<double, N_STATES, N_STATES> m_p_covariance;
  Matrix3d                           m_acc_bias_variance;
  Matrix3d                           m_gyro_bias_variance;

  // m_jacobian -> pg. 61 (271)
  Matrix<double, N_STATES, N_STATES_2> m_jacobian;

  const Matrix<double, 3, 3>               Identity3x3 = Matrix<double, 3, 3>::Identity();
  const Matrix<double, N_STATES, N_STATES> IdentityNxN =
    Matrix<double, N_STATES, N_STATES>::Identity();

  void inject_error_state(const Matrix<double, N_STATES, 1>& error_state);
  void inject_error_state_drift(const Matrix<double, N_STATES, 1>& error_state,
                                Vector3d&    sensor_translation_drift,
                                Quaterniond& sensor_rotation_drift);

public:
  /**
   * @brief Construct a new EsEkf2 object
   *
   * @param params EKF Parameters
   */
  EsEkf2(const EsEkfParams& params);

  /**
   * @brief Prediction step. Use imu to predict position, velocity and orientation.
   *
   * @param imu_f Linear acceleration from IMU.
   * @param var_imu_f Variance of the linear acceleration.
   * @param imu_w Angular velocity from IMU.
   * @param var_imu_w Variance of the angular velocity.
   * @param delta_t Time step.
   */
  void prediction(const Matrix<double, 3, 1>& imu_f,
                  const Matrix3d&             var_imu_f,
                  const Matrix<double, 3, 1>& imu_w,
                  const Matrix3d&             var_imu_w,
                  double                      delta_t);

  /**
   * @brief Measurement update. Use sensor measurement of pose to update the position
   * estimation and reduce uncertainty.
   *
   * @param R_cov Measurements covariance matrix.
   * @param measurements New measurements.
   */
  void poseMeasurementUpdate(const Matrix3d&             R_cov,
                             const Matrix<double, 3, 1>& measurements);

  /**
   * @brief Measurement update for the heading angle. Also performs a drift update if it
   * is a drift sensor. Use sensor measurement of pose to update the orientation
   * estimation and reduce uncertainty.
   *
   * @param R_cov Measurements covariance matrix.
   * @param measurements New measurements.
   * @param est_position_drift Current estimated position drift.
   * @param est_quaternion_drift Current estimated quaternion drift.
   * @param position_drift_cov
   * @param rotation_drift_cov
   */
  void angleMeasurementUpdateDrift(const Matrix<double, 4, 4>& R_cov,
                                   const Quaterniond&          measurements,
                                   Vector3d&                   est_position_drift,
                                   Quaterniond&                est_quaternion_drift,
                                   Matrix3d&                   position_drift_cov,
                                   Matrix3d&                   rotation_drift_cov);

  /**
   * @brief Measurement update for the heading angle.
   *
   * @param R_cov Measurement covariance matrix
   * @param measurements New orientation measurements.
   */
  void angleMeasurementUpdate(const Matrix<double, 4, 4>& R_cov,
                              const Quaterniond&          measurements);
  /**
   * @brief Measurement update including position drift. Use sensor measurement of pose to
   * update the position estimation and reduce uncertainty.
   *
   * @param R_cov Measurements covariance matrix.
   * @param measurements New measurements.
   * @param est_position_drift Current estimated position drift.
   * @param est_quaternion_drift Current estimated quaternion drift.
   * @param position_drift_cov
   * @param rotation_drift_cov
   */
  void poseMeasurementUpdateDrift(const Matrix3d&             R_cov,
                                  const Matrix<double, 3, 1>& measurements,
                                  Vector3d&                   est_position_drift,
                                  Quaterniond&                est_quaternion_drift,
                                  Matrix3d&                   position_drift_cov,
                                  Matrix3d&                   rotation_drift_cov);

  // Setters
  void setP(Matrix<double, 3, 1> p) { m_est_position = std::move(p); }
  void setV(Matrix<double, 3, 1> v) { m_est_lin_velocity = std::move(v); }
  void setWb(Matrix<double, 3, 1> wb) { m_est_gyro_bias = std::move(wb); }
  void setFb(Matrix<double, 3, 1> fb) { m_est_acc_bias = std::move(fb); }
  void setQ(Quaterniond q) { m_est_quaternion = std::move(q); }
  // Getters
  const Matrix<double, 10, 1> getState()
  {
    Matrix<double, 10, 1> state;
    state << m_est_position, m_est_lin_velocity, m_est_quaternion.w(),
      m_est_quaternion.x(), m_est_quaternion.y(), m_est_quaternion.z();
    return state;
  }
  const Vector3d&    getP() const { return m_est_position; }
  const Vector3d&    getLinVelocity() const { return m_est_lin_velocity; }
  const Quaterniond& getOrientation() const { return m_est_quaternion; }
  const Matrix3d     getPositionCov() const
  {
    return m_p_covariance.block<3, 3>(POSITION_IDX, POSITION_IDX);
  }
  const Matrix3d getLinVelocityCov() const
  {
    return m_p_covariance.block<3, 3>(VELOCITY_IDX, VELOCITY_IDX);
  }
  const Matrix3d getOrientationCov() const
  {
    return m_p_covariance.block<3, 3>(ANGLE_AXIS_IDX, ANGLE_AXIS_IDX);
  }
};

#endif// SENSOR_FUSION_FILTER_H

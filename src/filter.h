#ifndef SENSOR_FUSION_FILTER_H
#define SENSOR_FUSION_FILTER_H

#include <Eigen/Geometry>
#include <iostream>

#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "structures.h"
#include "utils.h"

// nuber of error states
#define N_STATES 24// 18

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
  Translation3d                      m_est_position;
  Translation3d                      m_est_lin_velocity;
  Translation3d                      m_est_gyro_bias;
  Translation3d                      m_est_acc_bias;
  Translation3d                      m_est_gravity;
  Quaterniond                        m_est_quaternion;
  Matrix<double, N_STATES, N_STATES> m_p_covariance;
  Matrix3d                           m_acc_bias_variance;
  Matrix3d                           m_gyro_bias_variance;

  // m_jacobian -> pg. 61 (271)
  Matrix<double, N_STATES, 12> m_jacobian;

  const Matrix<double, 3, 3>               Identity3x3 = Matrix<double, 3, 3>::Identity();
  const Matrix<double, N_STATES, N_STATES> IdentityNxN =
    Matrix<double, N_STATES, N_STATES>::Identity();

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
                  const double                delta_t);

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
   */
  void angleMeasurementUpdateDrift(const Matrix<double, 4, 4>& R_cov,
                              const Quaterniond&          measurements,
                              Translation3d&              est_position_drift,
                              Quaterniond&                est_quaternion_drift);

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
   */
  void poseMeasurementUpdateDrift(const Matrix3d&             R_cov,
                                  const Matrix<double, 3, 1>& measurements,
                                  Translation3d&              est_position_drift,
                                  Quaterniond&                est_quaternion_drift);

  // Setters
  void setP(Matrix<double, 3, 1> p) { m_est_position.vector() = std::move(p); }
  void setV(Matrix<double, 3, 1> v) { m_est_lin_velocity.vector() = std::move(v); }
  void setWb(Matrix<double, 3, 1> wb) { m_est_gyro_bias.vector() = std::move(wb); }
  void setFb(Matrix<double, 3, 1> fb) { m_est_acc_bias.vector() = std::move(fb); }
  void setQ(Matrix<double, 4, 1> q)
  {
    m_est_quaternion.w() = q[0];
    m_est_quaternion.x() = q[1];
    m_est_quaternion.y() = q[2];
    m_est_quaternion.z() = q[3];
    ROS_INFO_STREAM("EsEkf2::setQ() - q = " << q.transpose());
  }
  // Getters
  const Matrix<double, 10, 1> getState()
  {
    Matrix<double, 10, 1> state;
    state << m_est_position.vector(), m_est_lin_velocity.vector(), m_est_quaternion.w(),
      m_est_quaternion.x(), m_est_quaternion.y(), m_est_quaternion.z();
    return state;
  }
  const Vector3d&    getP() const { return m_est_position.vector(); }
  const Vector3d&    getLinVelocity() const { return m_est_lin_velocity.vector(); }
  const Quaterniond& getOrientation() const { return m_est_quaternion; }
  const Matrix3d     getPositionCov() const { return m_p_covariance.block<3, 3>(0, 0); }
  const Matrix3d getLinVelocityCov() const { return m_p_covariance.block<3, 3>(3, 3); }
  const Matrix3d getOrientationCov() const { return m_p_covariance.block<3, 3>(6, 6); }
};

#endif// SENSOR_FUSION_FILTER_H

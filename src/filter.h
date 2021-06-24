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
 */

using namespace Eigen;
class EsEkf2
{
private:
  // State (pose,lin vel, gyroscope bias,
  // accelerometer bias, gravity)
  Translation3d p_est, v_est, wb_est, fb_est, g_est;
  // Orientation in quaternions
  Quaterniond q_est, q_drift;
  // State covariance matrix
  Matrix<double, N_STATES, N_STATES> p_cov;
  Matrix<double, N_STATES, 12>       l_jac;
  bool                               init;
  Translation3d                      p_drift;
  Matrix3d                           var_imu_fb, var_imu_wb;
  const Matrix<double, 3, 3>         I3x3 = Matrix<double, 3, 3>::Identity();

public:
  // Constructor
  // Set gravity vector and bias vectors
  EsEkf2(const EsEkfParams& params);
  /*
   * The prediction step. Here we use imu measurements to get an
   * estimate of the pose, velocity and orientation.
   */
  void prediction(const Matrix<double, 3, 1>& imu_f,
                  const Matrix3d&             var_imu_f,
                  const Matrix<double, 3, 1>& imu_w,
                  const Matrix3d&             var_imu_w,
                  const double                delta_t);
  /*
   * Measurement update. Use sensor measurement of pose to
   * update the estimation and reduce uncertainty.
   */
  void poseMeasurementUpdate(const Matrix3d& R_cov, const Matrix<double, 3, 1>& y);
  void angleMeasurementUpdate(const Matrix<double, 4, 4>& R_cov, const Quaterniond& y);
  void poseMeasurementUpdateDrift(const Matrix3d& R_cov, const Matrix<double, 3, 1>& y);

  // Setters
  void setP(Matrix<double, 3, 1> p) { p_est.vector() = std::move(p); }
  void setV(Matrix<double, 3, 1> v) { v_est.vector() = std::move(v); }
  void setWb(Matrix<double, 3, 1> wb) { wb_est.vector() = std::move(wb); }
  void setFb(Matrix<double, 3, 1> fb) { fb_est.vector() = std::move(fb); }
  void setQ(Matrix<double, 4, 1> q)
  {
    q_est.w() = q[0];
    q_est.x() = q[1];
    q_est.y() = q[2];
    q_est.z() = q[3];
    ROS_INFO_STREAM("EsEkf2::setQ() - q = " << q.transpose());
  }
  // Getters
  Matrix<double, 10, 1> getState()
  {
    Matrix<double, 10, 1> state;
    state << p_est.vector(), v_est.vector(), q_est.w(), q_est.x(), q_est.y(), q_est.z();
    return state;
  }
  Matrix<double, 3, 1> getP() { return p_est.vector(); }
  Matrix<double, 3, 1> getPDrift() { return p_drift.vector(); }
  Matrix3d             getQDrift() { return q_drift.toRotationMatrix(); }
};

#endif// SENSOR_FUSION_FILTER_H

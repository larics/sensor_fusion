#ifndef SENSOR_FUSION_STRUCTURES_H
#define SENSOR_FUSION_STRUCTURES_H
#include <Eigen/Geometry>
#include <iostream>

#include "nav_msgs/Odometry.h"
#include "vector"

using namespace Eigen;

/*
 * Here we define 2 model covariance matrices.
 * -> Q_f is the model noise we get when we integrate
 * linear acceleration
 * -> Q_w model noise from angular velocity
 *
 * essentially imu noise for acceleration and
 * angular velocity measurement
 */
struct ModelCovariance
{
  Matrix<double, 3, 3> Q_f;
  Matrix<double, 3, 3> Q_w;
  ModelCovariance()
  {
    Q_f = MatrixXd::Zero(3, 3);
    Q_w = MatrixXd::Zero(3, 3);
  }
};

enum SensorMsgType {
  ODOMETRY                  = 0,
  TRANSFORM_STAMPED         = 1,
  POSE_STAMPED              = 2,
  IMU                       = 3,
  POSE_W_COVARIANCE_STAMPED = 4
};

enum SensorState {
  ORIENTATION_UPDATE           = 2,
  ORIENTATION_AND_DRIFT_UPDATE = 4,
  POSE_UPDATE                  = 8,
  POSE_AND_DRIFT_UPDATE        = 16,
  LIN_VELOCITY_UPDATE          = 32
};

struct SensorCovariance
{
  Matrix3d R_pose;
  Matrix3d R_orientation;
  SensorCovariance()
  {
    R_pose        = Matrix3d::Zero();
    R_orientation = Matrix3d::Zero();
  }
};
struct SensorParams
{
  std::string topic;
  std::string id;
  // Assume all sensors are position sensors unless specified otherwise
  bool                 is_position_sensor = true;
  bool                 is_orientation_sensor;
  bool                 is_velocity_sensor;
  bool                 estimate_drift;
  bool                 origin_at_first_measurement = false;
  int                  msg_type;
  Matrix<double, 3, 3> rotation_mat;
  Matrix<double, 3, 1> translation;
  SensorCovariance     cov;// correlation matrix of the sensor
  Vector3d             position_outlier_lim;
  Vector3d             veclocity_outlier_lim;
  Vector3d             orientation_outlier_lim;
};

struct OutlierChecks
{
  bool position_outlier         = false;
  bool drifted_position_outlier = false;
  bool lin_velocity_outlier     = false;
  bool orientation_outlier      = false;

  bool positionValid() const { return !position_outlier; }
  bool driftPositionValid() const { return !drifted_position_outlier; }
  bool linVelocityValid() const { return !lin_velocity_outlier; }
  bool orientationValid() const { return !orientation_outlier; }
};

struct EsEkfParams
{
  std::vector<SensorParams> sensors;
  std::string               initial_sensor_id;
  ModelCovariance           model;
  double                    estimation_frequncy;
  Translation3d             g;
  double                    init_pose_p_cov;
  double                    init_v_p_cov;
  double                    init_q_p_cov;
  double                    init_ab_p_cov;
  double                    init_wb_p_cov;
  double                    init_g_cov;
  double                    init_p_drift;
  double                    init_q_drift;
  bool                      estimate_acc_bias;
  bool                      estimate_gyro_bias;
  bool                      estimate_gravity;
  Matrix3d                  acc_bias_variance;
  Matrix3d                  gyro_bias_variance;
  double                    expected_imu_dt;

  EsEkfParams()
  {
    acc_bias_variance  = MatrixXd::Zero(3, 3);
    gyro_bias_variance = MatrixXd::Zero(3, 3);
    estimate_acc_bias  = true;
    estimate_gyro_bias = true;
    estimate_gravity   = true;
  }
};

#endif// SENSOR_FUSION_STRUCTURES_H

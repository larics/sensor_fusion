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
struct ModelCovariance {
  Matrix<double, 3, 3> Q_f, Q_w;
  ModelCovariance() {
    Q_f = MatrixXd::Zero(3, 3);
    Q_w = MatrixXd::Zero(3, 3);
  }
};

struct SensorCovariance {
  Matrix3d R_pose;
  Matrix<double, 4, 4> R_orientation;
  SensorCovariance() {
    R_pose = Matrix3d::Zero();
    R_orientation = MatrixXd::Zero(4, 4);
  }
};
struct SensorParams {
  std::string topic;
  std::string id;
  bool is_orientation_sensor;
  bool estimate_drift;
  bool origin_at_first_measurement = false;
  int msg_type;

  Matrix<double, 3, 3> rotation_mat;
  Matrix<double, 3, 1> translation;

  SensorCovariance cov;  // correlation matrix of the sensor
};



struct EsEkfParams {
  std::vector<SensorParams> sensors;
  ModelCovariance model;
  double outlier_constant;
  Translation3d g;

  double init_pose_p_cov, init_v_p_cov, init_q_p_cov, init_ab_p_cov,
         init_wb_p_cov, init_g_cov, init_p_drift, init_q_drift;

  bool use_cam_imu;
  bool estimate_acc_bias,
      estimate_gyro_bias,
      estimate_gravity;

  Matrix3d fb_var, wb_var;

  EsEkfParams() {
    fb_var = MatrixXd::Zero(3, 3);
    wb_var = MatrixXd::Zero(3, 3);
    estimate_acc_bias = true;
    estimate_gyro_bias = true;
    estimate_gravity = true;
  }
};

#endif  // SENSOR_FUSION_STRUCTURES_H

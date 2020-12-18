#ifndef SENSOR_FUSION_PARSE_YAML_H
#define SENSOR_FUSION_PARSE_YAML_H

#include <iostream>

#include "structures.h"
#include "yaml-cpp/yaml.h"

EsEkfParams parse_yaml(std::string config_file) {
  const YAML::Node config = YAML::LoadFile(config_file);

  EsEkfParams params;
  params.model.Q_f(0, 0) = config["Q_acc_x"].as<double>();
  params.model.Q_f(1, 1) = config["Q_acc_y"].as<double>();
  params.model.Q_f(2, 2) = config["Q_acc_z"].as<double>();
  params.model.Q_w(0, 0) = config["Q_angular_x"].as<double>();
  params.model.Q_w(1, 1) = config["Q_angular_y"].as<double>();
  params.model.Q_w(2, 2) = config["Q_angular_z"].as<double>();

  // vector of all sensor parameters
  SensorParams sensor;
  std::vector<double> R, translation, rotation;
  // sensor names and number of sensors
  std::vector<std::string> id =
      config["Sensor_prefix"].as<std::vector<std::string>>();
  for (size_t i = 0; i < id.size(); i++) {
    sensor.topic = config[id.at(i) + "_topic"].as<std::string>();
    sensor.id = id.at(i);

    /*
     * Currently the covariance matrix is expected to be
     * constant and diagonal
     */
    R = config[id.at(i) + "_R_pose"].as<std::vector<double>>();
    for (int j = 0; j < 3; ++j) {
      sensor.cov.R_pose(j, j) = R[j];
    }
    int is_orient_sensor = config[id.at(i) + "_is_orientation_sensor"]
                               .as<int>();
    sensor.is_orientation_sensor = is_orient_sensor;
    if (is_orient_sensor){
      R = config[id.at(i) + "_R_angle"].as<std::vector<double>>();
      for (int j = 0; j < 4; ++j) {
        sensor.cov.R_orientation(j, j) = R[j];
      }
    }
    /*
     * To get transforamtion from sensor link to base link
     */
    rotation = config[id.at(i) + "_rotation"].as<std::vector<double>>();
    Matrix<double, 3, 3> Rot(rotation.data());
    sensor.rotation_mat = Rot;

    translation = config[id.at(i) + "_translation"].as<std::vector<double>>();
    sensor.translation << translation.at(0), translation.at(1),
        translation.at(2);
    std::cout << "id.at(i)-> " << id.at(i) << '\n';
    sensor.estimate_drift = config[id.at(i) + "_estimate_drift"].as<int>();
    sensor.msg_type = config[id.at(i) + "_msg_type"].as<int>();
    params.sensors.push_back(sensor);
  }

  params.outlier_constant = config["outlier_constant"].as<double>();
  params.estimate_acc_bias = config["estimate_acc_bias"].as<int>();
  if (params.estimate_acc_bias){
    params.init_ab_p_cov = config["acc_bias_p_cov"].as<double>();
    translation = config["acc_bias_var"].as<std::vector<double>>();
    params.fb_var = Matrix3d::Zero();
    for (int i = 0; i < 3; ++i) {
      params.fb_var(i,i) = translation.at(i);
    }
  }
  params.estimate_gyro_bias = config["estimate_gyro_bias"].as<int>();
  if (params.estimate_gyro_bias){
    params.init_wb_p_cov = config["gyro_bias_p_cov"].as<double>();
    translation = config["gyro_bias_var"].as<std::vector<double>>();
    params.wb_var = Matrix3d::Zero();
    for (int i = 0; i < 3; ++i) {
      params.wb_var(i,i) = translation.at(i);
    }
  }
  params.estimate_gravity = config["estimate_gravity"].as<int>();
  params.init_g_cov = config["g_p_cov"].as<double>();
  translation = config["g"].as<std::vector<double>>();
  params.g.x() = translation.at(0);
  params.g.y() = translation.at(1);
  params.g.z() = translation.at(2);

  params.init_pose_p_cov = config["pose_p_cov"].as<double>();
  params.init_v_p_cov = config["vel_p_cov"].as<double>();
  params.init_q_p_cov = config["angle_p_cov"].as<double>();

  params.init_p_drift = config["p_drift_p_cov"].as<double>();
  params.init_q_drift = config["q_drift_p_cov"].as<double>();
  params.use_cam_imu = config["use_cam_imu"].as<int>();
  return params;
}

#endif  // SENSOR_FUSION_PARSE_YAML_H

#ifndef SENSOR_FUSION_PARSE_YAML_H
#define SENSOR_FUSION_PARSE_YAML_H

#include "structures.h"
#include "yaml-cpp/yaml.h"

EsEkfParams parse_yaml(const std::string& config_file)
{
  const YAML::Node config = YAML::LoadFile(config_file);

  EsEkfParams params;
  params.estimation_frequncy = config["estimation_frequency"].as<double>();
  params.model.Q_f(0, 0)     = config["Q_acc_x"].as<double>();
  params.model.Q_f(1, 1)     = config["Q_acc_y"].as<double>();
  params.model.Q_f(2, 2)     = config["Q_acc_z"].as<double>();
  params.model.Q_w(0, 0)     = config["Q_angular_x"].as<double>();
  params.model.Q_w(1, 1)     = config["Q_angular_y"].as<double>();
  params.model.Q_w(2, 2)     = config["Q_angular_z"].as<double>();
  params.initial_sensor_id   = config["initial_sensor"].as<std::string>();

  std::vector<std::string> id = config["Sensor_prefix"].as<std::vector<std::string>>();
  for (size_t i = 0; i < id.size(); i++) {
    SensorParams sensor;
    sensor.id    = id.at(i);
    sensor.topic = config[sensor.id + "_topic"].as<std::string>();

    /*
     * Currently the covariance matrix is expected to be
     * constant and diagonal
     */
    auto R = config[sensor.id + "_R_pose"].as<std::vector<double>>();
    for (int j = 0; j < 3; ++j) { sensor.cov.R_pose(j, j) = R[j]; }
    sensor.is_orientation_sensor = config[sensor.id + "_is_orientation_sensor"].as<int>();
    sensor.is_velocity_sensor    = config[sensor.id + "_is_velocity_sensor"].as<int>();
    if (sensor.is_orientation_sensor) {
      R = config[sensor.id + "_R_angle"].as<std::vector<double>>();
      for (int j = 0; j < 4; ++j) { sensor.cov.R_orientation(j, j) = R[j]; }
    }
    /*
     * To get transforamtion from sensor link to base link
     */
    auto rotation       = config[sensor.id + "_rotation"].as<std::vector<double>>();
    sensor.rotation_mat = Matrix3d(rotation.data());

    auto sensor_trans = config[sensor.id + "_translation"].as<std::vector<double>>();
    sensor.translation << sensor_trans.at(0), sensor_trans.at(1), sensor_trans.at(2);
    sensor.estimate_drift = config[sensor.id + "_estimate_drift"].as<int>();
    sensor.origin_at_first_measurement =
      config[sensor.id + "_origin_at_first_measurement"].as<int>();
    sensor.msg_type = config[sensor.id + "_msg_type"].as<int>();

    // Get outliers
    auto position_outlier_lim =
      config[sensor.id + "_position_outlier_lim"].as<std::vector<double>>();
    sensor.position_outlier_lim = Vector3d(position_outlier_lim.data());

    if (sensor.is_orientation_sensor) {
      auto orientation_outlier_lim =
        config[sensor.id + "_orientation_outlier_lim"].as<std::vector<double>>();
      sensor.orientation_outlier_lim = Vector3d(orientation_outlier_lim.data());
    }

    if (sensor.is_velocity_sensor) {
      auto velocity_outlier_lim =
        config[sensor.id + "_velocity_outlier_lim"].as<std::vector<double>>();
      sensor.veclocity_outlier_lim = Vector3d(velocity_outlier_lim.data());
    }

    params.sensors.push_back(sensor);
  }

  params.estimate_acc_bias = config["estimate_acc_bias"].as<int>();
  if (params.estimate_acc_bias) {
    params.init_ab_p_cov     = config["acc_bias_p_cov"].as<double>();
    auto acc_bias_var        = config["acc_bias_var"].as<std::vector<double>>();
    params.acc_bias_variance = Matrix3d::Zero();
    for (int i = 0; i < 3; ++i) { params.acc_bias_variance(i, i) = acc_bias_var.at(i); }
  }
  params.estimate_gyro_bias = config["estimate_gyro_bias"].as<int>();
  if (params.estimate_gyro_bias) {
    params.init_wb_p_cov      = config["gyro_bias_p_cov"].as<double>();
    auto gyro_bias_var        = config["gyro_bias_var"].as<std::vector<double>>();
    params.gyro_bias_variance = Matrix3d::Zero();
    for (int i = 0; i < 3; ++i) { params.gyro_bias_variance(i, i) = gyro_bias_var.at(i); }
  }
  params.estimate_gravity = config["estimate_gravity"].as<int>();
  params.init_g_cov       = config["g_p_cov"].as<double>();
  auto g_trans            = config["g"].as<std::vector<double>>();
  params.g.x()            = g_trans.at(0);
  params.g.y()            = g_trans.at(1);
  params.g.z()            = g_trans.at(2);

  params.init_pose_p_cov = config["pose_p_cov"].as<double>();
  params.init_v_p_cov    = config["vel_p_cov"].as<double>();
  params.init_q_p_cov    = config["angle_p_cov"].as<double>();

  params.init_p_drift = config["p_drift_p_cov"].as<double>();
  params.init_q_drift = config["q_drift_p_cov"].as<double>();
  return params;
}

#endif// SENSOR_FUSION_PARSE_YAML_H

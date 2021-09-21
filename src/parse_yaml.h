#ifndef SENSOR_FUSION_PARSE_YAML_H
#define SENSOR_FUSION_PARSE_YAML_H

// clang-format off
#if ENABLE_ASSERT == 1
#define MY_ASSERT(A) if (!(A)) { ROS_FATAL_STREAM("[" << __FILE__ << "]:" << __LINE__ << " - Assert failed!\n"); throw std::runtime_error("Failed parameter initialization."); } 
#else
#define MY_ASSERT(A)
#endif
// clang-format on

#include <ros/ros.h>
#include "structures.h"

template<class T> std::ostream& operator<<(std::ostream& o, const std::vector<T>& vec)
{
  o << "[";
  for (const auto& el : vec) { o << el << ", "; }
  o << "]";
  return o;
}

/**
 * @brief Attempt to get a value from the ROS parameter server or throws a runtime_error.
 *
 * @tparam T Value Typename
 * @param t_nh A ROS node handle
 * @param t_paramName ROS Parameter name.
 * @param t_paramContainer Variable where the parameter value will be saved.
 */
template<class T>
void getParamOrThrow(ros::NodeHandle&   t_nh,
                     const std::string& t_paramName,
                     T&                 t_paramContainer)
{
  bool gotParam = t_nh.getParam(t_paramName, t_paramContainer);
  ROS_INFO_STREAM("Got param [" << t_paramName << "] = " << t_paramContainer);
  if (!gotParam) {
    ROS_FATAL_STREAM("Unable to get param: [" << t_paramName << "]. Throwing...");
    throw std::runtime_error("Parameter initialization failed.");
  }
}

/**
 * @brief Attemps to return a value from the ROS parameter server or throws a
 * runtime_error.
 *
 * @tparam T Value Typename
 * @param nh A ROS node handle
 * @param param_name ROS parameter name
 * @return T
 */
template<class T> T getParamOrThrow(ros::NodeHandle& nh, std::string param_name)
{
  T param;
  getParamOrThrow(nh, param_name, param);
  return param;
}

namespace sf_params {
EsEkfParams get_rosparam(ros::NodeHandle& private_nh)
{
  EsEkfParams params;
  getParamOrThrow(private_nh, "expected_imu_dt", params.expected_imu_dt);
  getParamOrThrow(private_nh, "estimation_frequency", params.estimation_frequncy);
  getParamOrThrow(private_nh, "Q_acc_x", params.model.Q_f(0, 0));
  getParamOrThrow(private_nh, "Q_acc_y", params.model.Q_f(1, 1));
  getParamOrThrow(private_nh, "Q_acc_z", params.model.Q_f(2, 2));
  getParamOrThrow(private_nh, "Q_angular_x", params.model.Q_w(0, 0));
  getParamOrThrow(private_nh, "Q_angular_y", params.model.Q_w(1, 1));
  getParamOrThrow(private_nh, "Q_angular_z", params.model.Q_w(2, 2));
  getParamOrThrow(private_nh, "initial_sensor", params.initial_sensor_id);
  getParamOrThrow(private_nh, "estimate_acc_bias", params.estimate_acc_bias);
  getParamOrThrow(private_nh, "estimate_gyro_bias", params.estimate_gyro_bias);
  getParamOrThrow(private_nh, "estimate_gravity", params.estimate_gravity);
  getParamOrThrow(private_nh, "pose_p_cov", params.init_pose_p_cov);
  getParamOrThrow(private_nh, "vel_p_cov", params.init_v_p_cov);
  getParamOrThrow(private_nh, "angle_p_cov", params.init_q_p_cov);
  getParamOrThrow(private_nh, "p_drift_p_cov", params.init_p_drift);
  getParamOrThrow(private_nh, "q_drift_p_cov", params.init_q_drift);

  if (params.estimate_acc_bias) {
    getParamOrThrow(private_nh, "acc_bias_p_cov", params.init_ab_p_cov);
    params.acc_bias_variance = Matrix3d::Zero();
    auto acc_bias_variance =
      getParamOrThrow<std::vector<double>>(private_nh, "acc_bias_var");
    MY_ASSERT(acc_bias_variance.size() == 3);
    params.acc_bias_variance.diagonal() = Vector3d(acc_bias_variance.data());
  }

  if (params.estimate_gyro_bias) {
    getParamOrThrow(private_nh, "gyro_bias_p_cov", params.init_wb_p_cov);
    params.gyro_bias_variance = Matrix3d::Zero();
    auto gyro_bias_variance =
      getParamOrThrow<std::vector<double>>(private_nh, "gyro_bias_var");
    MY_ASSERT(gyro_bias_variance.size() == 3);
    params.gyro_bias_variance.diagonal() = Vector3d(gyro_bias_variance.data());
  }

  if (params.estimate_gravity) {
    getParamOrThrow(private_nh, "g_p_cov", params.init_g_cov);
  }
  {
    auto g = getParamOrThrow<std::vector<double>>(private_nh, "g");
    MY_ASSERT(g.size() == 3);
    params.g.vector() = Vector3d(g.data());
  }

  auto ids = getParamOrThrow<std::vector<std::string>>(private_nh, "Sensor_prefix");
  for (const auto& id : ids) {
    SensorParams sensor_params;
    sensor_params.id = id;
    getParamOrThrow(private_nh, id + "_topic", sensor_params.topic);

    {
      auto R_pose = getParamOrThrow<std::vector<double>>(private_nh, id + "_R_pose");
      MY_ASSERT(R_pose.size() == 3);
      sensor_params.cov.R_pose.diagonal() = Vector3d(R_pose.data());
    }

    // Check if it's an orientation sensor
    getParamOrThrow(
      private_nh, id + "_is_orientation_sensor", sensor_params.is_orientation_sensor);
    if (sensor_params.is_orientation_sensor) {
      auto R_orientation =
        getParamOrThrow<std::vector<double>>(private_nh, id + "_R_angle");
      MY_ASSERT(R_orientation.size() == 3);
      sensor_params.cov.R_orientation.diagonal() = Vector3d(R_orientation.data());
    }

    getParamOrThrow(
      private_nh, id + "_is_velocity_sensor", sensor_params.is_velocity_sensor);
    // TODO(lmark): Load additional velocity parameters

    // Fixed Transformation
    {
      auto rotation = getParamOrThrow<std::vector<double>>(private_nh, id + "_rotation");
      MY_ASSERT(rotation.size() == 9);
      sensor_params.rotation_mat = Eigen::Matrix3d(rotation.data());
      // This matrix should probably be transposed, it's initialized in column major order
      ROS_INFO_STREAM("Sensor " << id
                                << " fixed rotation is: " << sensor_params.rotation_mat);
    }

    {
      auto translation =
        getParamOrThrow<std::vector<double>>(private_nh, id + "_translation");
      MY_ASSERT(translation.size() == 3);
      sensor_params.translation = Eigen::Vector3d(translation.data());
      ROS_INFO_STREAM("sensor "
                      << id << " fixed translation is: " << sensor_params.translation);
    }

    getParamOrThrow(private_nh, id + "_estimate_drift", sensor_params.estimate_drift);
    getParamOrThrow(private_nh,
                    id + "_origin_at_first_measurement",
                    sensor_params.origin_at_first_measurement);
    getParamOrThrow(private_nh, id + "_msg_type", sensor_params.msg_type);

    // Get outliers limits
    {
      auto position_outlier_lim =
        getParamOrThrow<std::vector<double>>(private_nh, id + "_position_outlier_lim");
      MY_ASSERT(position_outlier_lim.size() == 3);
      sensor_params.position_outlier_lim = Eigen::Vector3d(position_outlier_lim.data());
    }

    if (sensor_params.is_orientation_sensor) {
      auto orientation_outlier_lim =
        getParamOrThrow<std::vector<double>>(private_nh, id + "_orientation_outlier_lim");
      MY_ASSERT(orientation_outlier_lim.size() == 3);
      sensor_params.orientation_outlier_lim =
        Eigen::Vector3d(orientation_outlier_lim.data());
    }

    if (sensor_params.is_velocity_sensor) {
      auto velocity_outlier_lim =
        getParamOrThrow<std::vector<double>>(private_nh, id + "_velocity_outlier_lim");
      MY_ASSERT(velocity_outlier_lim.size() == 3);
      sensor_params.veclocity_outlier_lim = Eigen::Vector3d(velocity_outlier_lim.data());
    }

    params.sensors.push_back(sensor_params);
  }

  return params;
}
}// namespace sf_params

#endif// SENSOR_FUSION_PARSE_YAML_H

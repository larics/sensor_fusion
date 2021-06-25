#ifndef SENSOR_FUSION_IMU_H
#define SENSOR_FUSION_IMU_H
#include <Eigen/Geometry>

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"

/*
 * Defines the imu subscriber and when an Imu msg comes
 * it calls a model prediction step of the es-ekf.
 */
class Imu
{

public:
  Imu(const ModelCovariance& cov, ros::NodeHandle& nh_private)
    : m_is_initialized(false), m_fresh_measurement(false), m_delta_t(0.0), Q_f_(cov.Q_f),
      Q_w_(cov.Q_w)
  {
    std::string imu_topic;
    nh_private.getParam("imu_topic", imu_topic);
    m_imu_sub = m_node_handle.subscribe(imu_topic, 1, &Imu::callback, this);
  }

  void callback(const sensor_msgs::ImuPtr& msg)
  {
    if (!m_is_initialized) {
      ROS_INFO("Imu::callback() - Imu Prediction: init");
      m_is_initialized = true;
      m_old_time       = msg->header.stamp.toSec();
    } else {
      m_fresh_measurement = true;

      m_imu_f << msg->linear_acceleration.x, msg->linear_acceleration.y,
        msg->linear_acceleration.z;
      m_imu_w << msg->angular_velocity.x, msg->angular_velocity.y,
        msg->angular_velocity.z;

      m_delta_t  = msg->header.stamp.toSec() - m_old_time;
      m_old_time = msg->header.stamp.toSec();
    }
  }

  void setQ(Matrix<double, 3, 3> Q_f, Matrix<double, 3, 3> Q_w)
  {
    Q_w_ = std::move(Q_w);
    Q_f_ = std::move(Q_f);
  }

  const Matrix<double, 3, 1>& get_acc()
  {
    m_fresh_measurement = false;
    return m_imu_f;
  }

  const Matrix<double, 3, 1>& get_angular_vel()
  {
    m_fresh_measurement = false;
    return m_imu_w;
  }

  double getDeltaT() { return m_delta_t; }
  bool   isInit() { return m_is_initialized; }
  bool   newMeasurement() { return m_fresh_measurement; }

private:
  Matrix<double, 3, 3> Q_f_;
  Matrix<double, 3, 3> Q_w_;
  bool                 m_is_initialized;
  bool                 m_fresh_measurement;
  Matrix<double, 3, 1> m_imu_f;
  Matrix<double, 3, 1> m_imu_w;
  double               m_delta_t;
  double               m_old_time;
  ros::Subscriber      m_imu_sub;
  ros::NodeHandle      m_node_handle;
};

#endif// SENSOR_FUSION_IMU_H

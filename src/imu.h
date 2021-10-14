#ifndef SENSOR_FUSION_IMU_H
#define SENSOR_FUSION_IMU_H
#include <Eigen/Geometry>

#include "ros/duration.h"
#include "ros/forwards.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "structures.h"

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
    m_watchdog_timer =
      m_node_handle.createTimer(ros::Duration(TIMEOUT), &Imu::watchdog_callback, this);
  }

  void watchdog_callback(const ros::TimerEvent& /* unused */)
  {
    auto time_diff = ros::Time::now().toSec() - m_last_message_time;
    if (time_diff > TIMEOUT) {
      m_sensor_responsive = false;
      ROS_FATAL("[Imu] unresponsive!");
    } else {
      m_sensor_responsive = true;
    }
  }

  bool isResponsive()
  {
    return m_sensor_responsive;
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
    
    m_last_message_time = ros::Time::now().toSec();
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
  static constexpr auto TIMEOUT = 0.5;

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
  double               m_last_message_time;
  bool                 m_sensor_responsive = false;
  ros::Timer           m_watchdog_timer;
};

#endif// SENSOR_FUSION_IMU_H

#ifndef SENSOR_FUSION_IMU_H
#define SENSOR_FUSION_IMU_H
#include <Eigen/Geometry>

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"

/*
 * Defines the imu subscriber and when an Imu msg comes
 * it calls a model prediction step of the es-ekf.
 */
class Imu {
  ros::Subscriber imu_sub;
  ros::NodeHandle node_handle_;

 public:
  Imu(ModelCovariance cov, ros::NodeHandle& nh_private)
      : init_(false),
        fresh_measurement_(false),
        delta_t_(0.0),
        Q_f_(cov.Q_f),
        Q_w_(cov.Q_w) {
    std::string imu_topic;
    nh_private.getParam("imu_topic", imu_topic);
    imu_sub = node_handle_.subscribe(imu_topic, 1, &Imu::callback, this);
  }
  void callback(const sensor_msgs::ImuPtr& msg) {
    if (!init_) {
      ROS_INFO("Imu Prediction: init");
      init_ = true;
      old_time = msg->header.stamp.toSec();
    } else {
      fresh_measurement_ = true;
      imu_f_ << msg->linear_acceleration.x, msg->linear_acceleration.y,
          msg->linear_acceleration.z;

      imu_w_ << msg->angular_velocity.x, msg->angular_velocity.y,
          msg->angular_velocity.z;

      delta_t_ = msg->header.stamp.toSec() - old_time;
      old_time = msg->header.stamp.toSec();
    }
  }

  void setQ(Matrix<double, 3, 3> Q_f, Matrix<double, 3, 3> Q_w) {
    Q_w_ = Q_w;
    Q_f_ = Q_f;
  }
  bool isInit() { return init_; }
  bool newMeasurement() { return fresh_measurement_; }
  Matrix<double, 3, 1> get_acc() {
    fresh_measurement_ = false;
    return imu_f_;
  }
  Matrix<double, 3, 1> get_angular_vel() {
    fresh_measurement_ = false;
    return imu_w_;
  }
  double getDeltaT() { return delta_t_; }

 private:
  // Noise matrix
  Matrix<double, 3, 3> Q_f_, Q_w_;

  // bool fresh measurement
  bool init_, fresh_measurement_;
  Matrix<double, 3, 1> imu_f_, imu_w_;
  double delta_t_;
  double old_time;
};

#endif  // SENSOR_FUSION_IMU_H

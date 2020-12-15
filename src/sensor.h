#ifndef SENSOR_FUSION_SENSOR_H
#define SENSOR_FUSION_SENSOR_H

#include <Eigen/Geometry>

#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "structures.h"

using namespace Eigen;

/*
 * We define sensor subscriber and when an odometry msg comes we
 * update the es-ekf state.
 */

class Sensor {
  // subscriber for sensor odometry type msg
  ros::Subscriber sensor_sub;
  ros::NodeHandle node_handle_;
  SensorParams params_;
  Translation3d pose_;
  Quaterniond quat_;

  bool fresh_measurement_;

 public:
  Sensor(SensorParams params) : params_(params), fresh_measurement_(false) {
    sensor_sub = node_handle_.subscribe(params_.topic, 1,
                                        &Sensor::callback_sensor, this);
  }
  void setR(Matrix3d R) { params_.cov.R = R; }

  void callback_sensor(const nav_msgs::OdometryPtr& msg) {
    fresh_measurement_ = true;

    pose_.x() = msg->pose.pose.position.x;
    pose_.y() = msg->pose.pose.position.y;
    pose_.z() = msg->pose.pose.position.z;

    quat_.w() = msg->pose.pose.orientation.w;
    quat_.x() = msg->pose.pose.orientation.x;
    quat_.y() = msg->pose.pose.orientation.y;
    quat_.z() = msg->pose.pose.orientation.z;
  }

  Matrix<double, 3, 1> getPose() {
    fresh_measurement_ = false;
    return (params_.rotation_mat * pose_).translation() + params_.translation;
  }

  Matrix<double, 4, 1> getOrientation() {
    // TODO add transformation
    fresh_measurement_ = false;
    return {quat_.w(), quat_.x(), quat_.y(), quat_.z()};
  }

  bool newMeasurement() { return fresh_measurement_; }
  bool isOrientationSensor() { return params_.is_orientation_sensor; }
  bool estimateDrift() { return params_.estimate_drift; }

  Matrix3d getRPose() { return params_.cov.R; }
  Matrix3d getROrientation() { return params_.cov.R_orientation; }

  ~Sensor() {
    std::cout << "SENSOR DESTRUCTOR" << '\n';
    std::string name = "sensor_" + params_.id;
  }
};
#endif
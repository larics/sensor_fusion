#ifndef SENSOR_FUSION_SENSOR_H
#define SENSOR_FUSION_SENSOR_H

#include <Eigen/Geometry>

#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "structures.h"
#include "geometry_msgs/TransformStamped.h"
#include "std_msgs/Bool.h"
#include <geometry_msgs/PoseStamped.h>

using namespace Eigen;

/*
 * We define sensor subscriber and when an odometry msg comes we
 * update the es-ekf state.
 */

//TODO add sensor_state publishing
class Sensor {
  // subscriber for sensor odometry type msg
  ros::Subscriber sensor_sub;
  ros::Publisher sensor_state_pub;
  ros::Publisher transformed_pub;
  ros::NodeHandle node_handle_;
  SensorParams params_;
  Translation3d pose_;
  Quaterniond quat_;

  bool fresh_measurement_;
  bool first_measurement_ = false;

 public:
  Sensor(SensorParams params) : params_(params), fresh_measurement_(false) {
    if (params.msg_type == 0){
      sensor_sub = node_handle_.subscribe(params_.topic, 1,
                                          &Sensor::callback_sensor, this); 
    }
    else if (params.msg_type == 1){
      sensor_sub = node_handle_.subscribe(params_.topic, 1,
                                          &Sensor::callback_sensor_pozyx, this);
    }
    sensor_state_pub = node_handle_.advertise<std_msgs::Bool>(params.id +
                                                              "_state", 1);
    transformed_pub = node_handle_.advertise<geometry_msgs::PoseStamped>(params.id + "_transformed_pose", 1);
    
    quat_.w() = 1;
    quat_.x() = 0;
    quat_.y() = 0;
    quat_.z() = 0;
  }
  void setR(Matrix3d R) { params_.cov.R_pose = R; }
  void publishState(bool state){
    std_msgs::Bool state_msg;
    state_msg.data = state;
    sensor_state_pub.publish(state_msg);
  }
  void publishTransformedPose(){
    geometry_msgs::PoseStamped transformed_msg;
    auto transformed_pose = getPose();
    transformed_msg.header.frame_id = "world";
    transformed_msg.header.stamp = ros::Time::now();
    transformed_msg.pose.position.x = transformed_pose.x();
    transformed_msg.pose.position.y = transformed_pose.y();
    transformed_msg.pose.position.z = transformed_pose.z();
    transformed_msg.pose.orientation.x = quat_.x();
    transformed_msg.pose.orientation.y = quat_.y();
    transformed_msg.pose.orientation.z = quat_.z();
    transformed_msg.pose.orientation.w = quat_.w();
    transformed_pub.publish(transformed_msg);    
  }
  void callback_sensor_pozyx(const geometry_msgs::TransformStamped& msg) {
    // ROS_INFO("camera_posix_callback");
    fresh_measurement_ = true;
    if (!first_measurement_ && params_.origin_at_first_measurement) {
      first_measurement_ = true;
      params_.translation.x() = - msg.transform.translation.x;
      params_.translation.y() = - msg.transform.translation.y;
      params_.translation.z() = - msg.transform.translation.z;
    }
    pose_.x() = msg.transform.translation.x;
    pose_.y() = msg.transform.translation.y;
    pose_.z() = msg.transform.translation.z;
  }

  Vector3d getDriftedPose(Matrix3d R, Vector3d d){
    return R.inverse() * pose_.translation() - R.inverse() * d;
  }

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
    return (params_.rotation_mat * pose_).translation() + params_.rotation_mat * params_.translation;
  }

  Quaterniond getOrientation() {
    // TODO add transformation
    fresh_measurement_ = false;
    return quat_;
  }
  Matrix<double, 4, 1> getOrientationVector() {
    fresh_measurement_ = false;
    return {quat_.w(), quat_.x(), quat_.y(), quat_.z()};
  }
  bool newMeasurement() { return fresh_measurement_; }
  bool isOrientationSensor() { return params_.is_orientation_sensor; }
  bool estimateDrift() { return params_.estimate_drift; }

  Matrix3d getRPose() { return params_.cov.R_pose; }
  Matrix<double,4,4> getROrientation() { return params_.cov.R_orientation; }

  ~Sensor() {
    std::cout << "SENSOR DESTRUCTOR" << '\n';
  }
};
#endif
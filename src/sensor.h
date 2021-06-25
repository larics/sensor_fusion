#ifndef SENSOR_FUSION_SENSOR_H
#define SENSOR_FUSION_SENSOR_H

#include <Eigen/Geometry>

#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "structures.h"
#include "geometry_msgs/TransformStamped.h"
#include "std_msgs/Int32.h"
#include <geometry_msgs/PoseStamped.h>

using namespace Eigen;
class Sensor;
using SensorPtr = std::shared_ptr<Sensor>;

/*
 * We define sensor subscriber and when an odometry msg comes we
 * update the es-ekf state.
 */

// TODO add sensor_state publishing
class Sensor
{
private:
  ros::Subscriber m_sensor_sub;
  ros::Publisher  m_sensor_state_pub;
  ros::Publisher  m_transformed_pub;
  ros::NodeHandle m_node_handle;
  SensorParams    m_sensor_params;
  Translation3d   m_sensor_position;
  Quaterniond     m_sensor_q;
  bool            m_fresh_measurement = false;
  bool            m_first_measurement = false;

public:
  Sensor(const SensorParams& params) : m_sensor_params(params)
  {
    if (m_sensor_params.msg_type == 0) {
      m_sensor_sub =
        m_node_handle.subscribe(m_sensor_params.topic, 1, &Sensor::callback_sensor, this);
    } else if (m_sensor_params.msg_type == 1) {
      m_sensor_sub = m_node_handle.subscribe(
        m_sensor_params.topic, 1, &Sensor::callback_sensor_pozyx, this);
    }
    m_sensor_state_pub =
      m_node_handle.advertise<std_msgs::Int32>(m_sensor_params.id + "_state", 1);
    m_transformed_pub = m_node_handle.advertise<geometry_msgs::PoseStamped>(
      m_sensor_params.id + "_transformed_pose", 1);

    m_sensor_q.w() = 1;
    m_sensor_q.x() = 0;
    m_sensor_q.y() = 0;
    m_sensor_q.z() = 0;
  }
  void setR(Matrix3d R) { m_sensor_params.cov.R_pose = R; }
  void publishState(int state)
  {
    std_msgs::Int32 state_msg;
    state_msg.data = state;
    m_sensor_state_pub.publish(state_msg);
  }
  void publishTransformedPose()
  {
    geometry_msgs::PoseStamped transformed_msg;
    auto                       transformed_pose = getPose();
    transformed_msg.header.frame_id             = "world";
    transformed_msg.header.stamp                = ros::Time::now();
    transformed_msg.pose.position.x             = transformed_pose.x();
    transformed_msg.pose.position.y             = transformed_pose.y();
    transformed_msg.pose.position.z             = transformed_pose.z();
    transformed_msg.pose.orientation.x          = m_sensor_q.x();
    transformed_msg.pose.orientation.y          = m_sensor_q.y();
    transformed_msg.pose.orientation.z          = m_sensor_q.z();
    transformed_msg.pose.orientation.w          = m_sensor_q.w();
    m_transformed_pub.publish(transformed_msg);
  }
  void callback_sensor_pozyx(const geometry_msgs::TransformStamped& msg)
  {
    // ROS_INFO("camera_posix_callback");
    m_fresh_measurement = true;
    if (!m_first_measurement && m_sensor_params.origin_at_first_measurement) {
      ROS_INFO_STREAM(getSensorID() << " origin initialized at first measurement");
      m_first_measurement             = true;
      m_sensor_params.translation.x() = msg.transform.translation.x;
      m_sensor_params.translation.y() = msg.transform.translation.y;
      m_sensor_params.translation.z() = msg.transform.translation.z;
    }
    m_sensor_position.x() = msg.transform.translation.x;
    m_sensor_position.y() = msg.transform.translation.y;
    m_sensor_position.z() = msg.transform.translation.z;
  }

  Vector3d getDriftedPose(Matrix3d R, Vector3d d)
  {
    return R.inverse() * m_sensor_position.translation() - R.inverse() * d;
  }

  void callback_sensor(const nav_msgs::OdometryPtr& msg)
  {
    m_fresh_measurement = true;

    if (!m_first_measurement && m_sensor_params.origin_at_first_measurement) {
      ROS_INFO_STREAM(getSensorID() << " origin initialized at first measurement");
      m_first_measurement             = true;
      m_sensor_params.translation.x() = msg->pose.pose.position.x;
      m_sensor_params.translation.y() = msg->pose.pose.position.y;
      m_sensor_params.translation.z() = msg->pose.pose.position.z;
    }

    m_sensor_position.x() = msg->pose.pose.position.x;
    m_sensor_position.y() = msg->pose.pose.position.y;
    m_sensor_position.z() = msg->pose.pose.position.z;

    m_sensor_q.w() = msg->pose.pose.orientation.w;
    m_sensor_q.x() = msg->pose.pose.orientation.x;
    m_sensor_q.y() = msg->pose.pose.orientation.y;
    m_sensor_q.z() = msg->pose.pose.orientation.z;
  }

  Matrix<double, 3, 1> getPose()
  {
    m_fresh_measurement = false;
    return (m_sensor_params.rotation_mat * m_sensor_position).translation()
           - m_sensor_params.rotation_mat * m_sensor_params.translation;
  }

  const Quaterniond& getOrientation()
  {
    // TODO add transformation
    m_fresh_measurement = false;
    return m_sensor_q;
  }

  Matrix<double, 4, 1> getOrientationVector()
  {
    m_fresh_measurement = false;
    return { m_sensor_q.w(), m_sensor_q.x(), m_sensor_q.y(), m_sensor_q.z() };
  }

  bool newMeasurement() const { return m_fresh_measurement; }
  bool isOrientationSensor() const { return m_sensor_params.is_orientation_sensor; }
  bool estimateDrift() const { return m_sensor_params.estimate_drift; }
  const std::string&          getSensorID() const { return m_sensor_params.id; }
  const Matrix3d&             getRPose() const { return m_sensor_params.cov.R_pose; }
  const Matrix<double, 4, 4>& getROrientation() const
  {
    return m_sensor_params.cov.R_orientation;
  }

  ~Sensor() { std::cout << "SENSOR DESTRUCTOR" << '\n'; }
};
#endif
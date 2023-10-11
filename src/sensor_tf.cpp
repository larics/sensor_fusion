#include "sensor_tf.h"
#include "Eigen/src/Geometry/Quaternion.h"

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_eigen/tf2_eigen.h>

sf::SensorTF::SensorTF(std::string base_link) : m_base_link(std::move(base_link))
{
  m_base_link_origin = m_base_link + "/robot";
}

void sf::SensorTF::publishSensorOrigin(const Sensor&      sensor,
                                       const Quaterniond& ekf_orientation,
                                       const Quaterniond& main_sensor_q)
{
  Eigen::Vector3d    position;
  Eigen::Quaterniond orientation;

  if (sensor.estimateDrift()) {
    position    = sensor.getDriftedPose();
    orientation = sensor.getDriftedRotation();
  } else {
    position    = sensor.getPose();
    orientation = sensor.getOrientation();
  }

  if (!sensor.isOrientationSensor()) { orientation = ekf_orientation; }


  // Get the inverse transformation
  tf2::Transform tf_inv(
    tf2::Quaternion(orientation.x(), orientation.y(), orientation.z(), orientation.w()),
    tf2::Vector3(position.x(), position.y(), position.z()));
  tf_inv = tf_inv.inverse();

  if (!std::isfinite(tf_inv.getRotation().x()) || !std::isfinite(tf_inv.getRotation().y())
      || !std::isfinite(tf_inv.getRotation().z())
      || !std::isfinite(tf_inv.getRotation().w())) {
    tf2::Quaternion q(0, 0, 0, 1);
    tf_inv.setRotation(q);
    ROS_ERROR_THROTTLE(
      5.0, "[SensorTF::publishSensorOrigin] - tf_inv is infinite, publishing [0,0,0,1]");
  }

  // Publish inverted sensor sensor origin
  geometry_msgs::TransformStamped tf;
  tf.header.stamp            = ros::Time::now();
  tf.header.frame_id         = m_base_link_origin;
  tf.child_frame_id          = m_base_link + "/" + sensor.getName();
  tf.transform.translation.x = tf_inv.getOrigin().x();
  tf.transform.translation.y = tf_inv.getOrigin().y();
  tf.transform.translation.z = tf_inv.getOrigin().z();

  auto q                  = tf_inv.getRotation();
  tf.transform.rotation.x = q.x();
  tf.transform.rotation.y = q.y();
  tf.transform.rotation.z = q.z();
  tf.transform.rotation.w = q.w();
  m_tf_broadcaster.sendTransform(tf);

  // Get raw sensor data
  auto&              raw_position    = sensor.getRawPosition();
  Eigen::Quaterniond raw_orientation = sensor.getRawOrientation();
  if (!sensor.isOrientationSensor()) { raw_orientation = main_sensor_q; }

  // Get the inverse transformation
  tf2::Transform tf_raw_inv(
    tf2::Quaternion(
      raw_orientation.x(), raw_orientation.y(), raw_orientation.z(), raw_orientation.w()),
    tf2::Vector3(raw_position.x(), raw_position.y(), raw_position.z()));
  tf_raw_inv = tf_raw_inv.inverse();

  if (!std::isfinite(tf_raw_inv.getRotation().x())
      || !std::isfinite(tf_raw_inv.getRotation().y())
      || !std::isfinite(tf_raw_inv.getRotation().z())
      || !std::isfinite(tf_raw_inv.getRotation().w())) {
    tf2::Quaternion q(0, 0, 0, 1);
    tf_raw_inv.setRotation(q);
    ROS_ERROR_THROTTLE(
      5.0,
      "[SensorTF::publishSensorOrigin] - tf_raw_inv is infinite, publishing [0,0,0,1]");
  }

  geometry_msgs::TransformStamped tf_raw;
  tf_raw.header.stamp    = ros::Time::now();
  tf_raw.header.frame_id = m_base_link_origin;
  tf_raw.child_frame_id  = m_base_link + "/" + sensor.getName() + "_raw";

  tf_raw.transform.translation.x = tf_raw_inv.getOrigin().x();
  tf_raw.transform.translation.y = tf_raw_inv.getOrigin().y();
  tf_raw.transform.translation.z = tf_raw_inv.getOrigin().z();

  auto q_raw                  = tf_raw_inv.getRotation();
  tf_raw.transform.rotation.x = q_raw.x();
  tf_raw.transform.rotation.y = q_raw.y();
  tf_raw.transform.rotation.z = q_raw.z();
  tf_raw.transform.rotation.w = q_raw.w();
  m_tf_broadcaster.sendTransform(tf_raw);
}

void sf::SensorTF::publishOrigin(const nav_msgs::Odometry& odom,
                                 const std::string&        tf_name)
{
  // Get the inverse transformation
  tf2::Transform tf_inv(
    tf2::Quaternion(odom.pose.pose.orientation.x,
                    odom.pose.pose.orientation.y,
                    odom.pose.pose.orientation.z,
                    odom.pose.pose.orientation.w),
    tf2::Vector3(
      odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z));
  tf_inv = tf_inv.inverse();

  // Publish inverted sensor sensor origin
  geometry_msgs::TransformStamped tf;
  tf.header.stamp            = ros::Time::now();
  tf.header.frame_id         = m_base_link_origin;
  tf.child_frame_id          = m_base_link + "/" + tf_name;
  tf.transform.translation.x = tf_inv.getOrigin().x();
  tf.transform.translation.y = tf_inv.getOrigin().y();
  tf.transform.translation.z = tf_inv.getOrigin().z();

  auto q                  = tf_inv.getRotation();
  tf.transform.rotation.x = q.x();
  tf.transform.rotation.y = q.y();
  tf.transform.rotation.z = q.z();
  tf.transform.rotation.w = q.w();
  m_tf_broadcaster.sendTransform(tf);
}

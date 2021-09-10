#include "sensor_tf.h"

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_eigen/tf2_eigen.h>

sf::SensorTF::SensorTF(std::string base_link) : m_base_link(std::move(base_link))
{
  m_base_link_origin = m_base_link + "/robot";
}

void sf::SensorTF::publishSensorOrigin(const Sensor& sensor, const Quaterniond& ekf_orientation)
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

  if (!sensor.isOrientationSensor()) {
    orientation = ekf_orientation;
  }

  // Get the inverse transformation
  tf2::Transform tf_inv(
    tf2::Quaternion(orientation.x(), orientation.y(), orientation.z(), orientation.w()),
    tf2::Vector3(position.x(), position.y(), position.z()));
  tf_inv = tf_inv.inverse();

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
#ifndef TF_BROADCASTER_H
#define TF_BROADCASTER_H

#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "sensor.h"

namespace sf {
class SensorTF
{
public:
  /**
   * @brief Construct a new Sensor TF object.
   *
   * @param base_link Name of the base link. Parent of all other sensor transformations.
   */
  explicit SensorTF(std::string base_link);

  /**
   * @brief Publish sensor origin as a child of the base link.
   *
   * @param sensor Ovbject type Sensor.
   */
  void publishSensorOrigin(const Sensor& sensor, const Quaterniond& ekf_orientation);

  /**
   * @brief Publish estimated origin from the odometry message.
   *
   * @param odom
   * @param name
   */
  void publishOrigin(const nav_msgs::Odometry& odom, const std::string& tf_name);

private:
  tf2_ros::TransformBroadcaster m_tf_broadcaster;
  std::string                   m_base_link;
  std::string                   m_base_link_origin;
};
}// namespace sf

#endif /* TF_BROADCASTER_H */
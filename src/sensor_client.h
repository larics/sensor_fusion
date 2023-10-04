#ifndef SENSOR_FUSION_SENSOR_CLIENT_H
#define SENSOR_FUSION_SENSOR_CLIENT_H

#include <Eigen/Geometry>

#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Bool.h"

#include "structures.h"
#include "imu.h"
#include "filter.cpp"
#include "sensor.h"
#include "sensor_tf.h"
#include <memory>
#include <map>

class SensorClient
{
public:
  SensorClient(const EsEkfParams& params,
               ros::NodeHandle&   nh_private,
               std::string        uav_name);
  SensorClient(const EsEkfParams& params, ros::NodeHandle& nh_private);

  void stateEstimation(const ros::TimerEvent& msg);
  void helper_odom_cb(const nav_msgs::Odometry& msg);

private:
  ros::Publisher                   m_estimate_pub;
  ros::Subscriber                  m_helper_odom_sub;
  nav_msgs::Odometry               m_helper_odom;
  ros::Timer                       m_update_timer;
  EsEkfParams                      m_ekf_params;
  EsEkf2                           m_es_ekf;
  Imu                              m_imu_sensor;
  std::map<std::string, SensorPtr> m_sensor_vector;
  ros::NodeHandle                  m_node_handle;
  bool                             m_start_flag;
  std::string                      m_uav_name;
  sf::SensorTF                     m_sensor_tf;

  std::string m_odom_helper_topic  = "unused/global_position/local";
  bool        m_odom_helper_enable = false;
};

#endif// SENSOR_FUSION_SENSOR_CLIENT_H

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
#include <memory>

class SensorClient
{
public:
  SensorClient(const EsEkfParams& params, ros::NodeHandle& nh_private);

  void stateEstimation(const ros::TimerEvent& msg);

private:
  ros::Publisher         m_estimate_pub;
  ros::Timer             m_update_timer;
  EsEkfParams            m_ekf_params;
  EsEkf2                 m_es_ekf;
  Imu                    m_imu_sensor;
  std::vector<SensorPtr> m_sensor_vector;
  ros::NodeHandle        m_node_handle;
  bool                   m_start_flag;
};

#endif// SENSOR_FUSION_SENSOR_CLIENT_H

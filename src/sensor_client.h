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

class SensorClient
{
public:
  SensorClient(const EsEkfParams& params, ros::NodeHandle& nh_private);

  void state_estimation(const ros::TimerEvent& msg);
  bool outlier_detection(const Matrix<double, 3, 1>& measurement);

private:
  ros::Subscriber camera_acc_sub_, camera_gyro_sub_, camera_odom_sub_, imu_sub_,
    posix_sub_, cartographer_sub_;

  ros::Publisher estimate_pub_, camera_state_pub_, pozyx_state_pub_,
    cartographer_state_pub_;
  ros::Timer           update_timer_;
  EsEkfParams          params_;
  EsEkf2               es_ekf_;
  Imu                  imu_;
  std::vector<Sensor*> sensor_vec_;

  ros::NodeHandle node_handle_;
  bool            start_flag_, start_imu_, start_camera_imu_;
  bool new_measurement_camera_odom_, new_measurement_posix_, new_measurement_camera_gyro_,
    new_measurement_camera_acc_, new_measurement_imu_;

  // acceleration and velocities
  Translation3d camera_acc_, camera_gyro_, imu_acc_, imu_gyro_;
  // Poses
  Translation3d camera_pose_, posix_pose_, cartographer_pose_;
  Translation3d camera_lin_vel_;
  Quaterniond   camera_orientation_;
  double        old_time_, delta_t_;
  double        outlier_constant_;
};

#endif// SENSOR_FUSION_SENSOR_CLIENT_H

#include <iostream>

#include "parse_yaml.h"
#include "ros/ros.h"
#include "sensor_client.cpp"

/*
 * Sensor fusion algorithm based on an error state kalman filter
 * This is the main function where we parse data and initialize the
 * filter. Also we set up all ros subscribers and publishers.
 */

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Sensor_Fusion");
  ros::NodeHandle node_handle;
  ros::NodeHandle nh_private("~");

  std::string config_file;
  std::string uav_name;
  bool        initialized = nh_private.getParam("config_yaml", config_file)
                     && nh_private.getParam("uav_name", uav_name);
  if (!initialized) {
    ROS_ERROR("[Sensor Fusion] Unable to initialize parameters. Exiting...");
    return 1;
  }

  // Next we parse data from the yaml file
  EsEkfParams params = sf_params::get_rosparam(nh_private);

  std::cout << "\n\nModel parameters:\n Q_f \n"
            << params.model.Q_f << "\n"
            << "Q_w\n"
            << params.model.Q_w << "\n\n"
            << "ES EKF params\n\n "
            << "Estimate acc bias: " << params.estimate_acc_bias << '\n'
            << "Estimate gyro bias: " << params.estimate_gyro_bias << '\n'
            << "Estimate gravity: " << params.estimate_gravity << '\n'
            << "Gravity vector: " << params.g.z() << '\n'
            << "intial P_cov\nPose:" << params.init_pose_p_cov << '\n'
            << "Vel: " << params.init_v_p_cov << '\n'
            << "Angle: " << params.init_q_p_cov << '\n'
            << "Acc bias: " << params.init_ab_p_cov << '\n'
            << "Gyro bias: " << params.init_wb_p_cov << '\n'
            << "Gravity: " << params.init_g_cov << '\n'
            << "Translation drift: " << params.init_p_drift << '\n'
            << "Rotatation drift: " << params.init_q_drift << '\n';

  for (const auto& sensor : params.sensors) {
    std::cout << "\n\nTopic: " << sensor.topic << "\n"
              << "Msg Type: " << sensor.msg_type << '\n'
              << "Position: " << sensor.is_position_sensor << "\n"
              << "Orientation: " << sensor.is_orientation_sensor << "\n"
              << "Drift: " << sensor.estimate_drift << "\n"
              << "R: \n"
              << sensor.cov.R_pose << "\n"
              << "Rotation:\n"
              << sensor.rotation_mat << "\n"
              << "Translation:\n"
              << sensor.translation << "\n  ";
  }

  SensorClient sensors(params, nh_private, uav_name);

  ROS_INFO("[main] - Starting sensor client");
  ros::MultiThreadedSpinner spinner(0);// Use max number of threads
  spinner.spin();// spin() will not return until the node has been shutdown

  return 0;
}

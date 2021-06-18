#include <iostream>

#include "parse_yaml.h"
#include "ros/ros.h"
#include "sensor_client.cpp"

/*
 * Sensor fusion algorithm based on an error state kalman filter
 * This is the main function where we parse data and initialize the
 * filter. Also we set up all ros subscribers and publishers.
 */

int main(int argc, char** argv) {
  ros::init(argc, argv, "Sensor_Fusion");
  ros::NodeHandle node_handle;
  ros::NodeHandle nh_private("~");

  // Next we parse data from the yaml file
  std::string config_file;
  nh_private.getParam("config_yaml", config_file);

  YAML::Node config = YAML::LoadFile(config_file);

  EsEkfParams params = parse_yaml(config_file);

  std::cout << "\n\nModel parameters:\n Q_f \n"
            << params.model.Q_f << "\n"
            << "Q_w\n"
            << params.model.Q_w << "\n\n"
            << "ES EKF params\n\n "
            << "Use camera imu: " << params.use_cam_imu << '\n'
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

  for (int i = 0; i < params.sensors.size(); ++i) {
    std::cout << "\n\nTopic: " << params.sensors.at(i).topic << "\n"
              << "Msg Type: " << params.sensors.at(i).msg_type << '\n'
              << "Orientation: " << params.sensors.at(i)
                                        .is_orientation_sensor << "\n"
              << "Drift: " << params.sensors.at(i).estimate_drift << "\n"
              << "R: \n"
              << params.sensors.at(i).cov.R_pose << "\n"
              << "Rotation:\n"
              << params.sensors.at(i).rotation_mat << "\n"
              << "Translation:\n"
              << params.sensors.at(i).translation << "\n  ";
  }

//TODO IZBRISI
  // EsEkf esEkf;
  // Camera camera(params,&esEkf,nh_private);

  // EsEkf2 esEkf2(params);
  SensorClient sensors(params, nh_private);

  ROS_INFO("Starting sensor client");
  ros::MultiThreadedSpinner spinner(0);  // Use max number of threads
  spinner.spin();  // spin() will not return until the node has been shutdown
  
  return 0;
}

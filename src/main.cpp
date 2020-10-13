
#include <iostream>
#include "ros/ros.h"
#include "RosClient.hpp"
#include "yaml-cpp/yaml.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "test");
  ros::NodeHandle node_handle;
  ros::NodeHandle nh_private("~");
  std::string config_file;
  nh_private.getParam("config_yaml", config_file);

  YAML::Node config = YAML::LoadFile(config_file);

  VehicleParams params;
  params.m = config["mass"].as<double>();
  params.g = config["g"].as<double>();
  params.Ixx = config["Ixx"].as<double>();
  params.Iyy = config["Iyy"].as<double>();
  params.Izz = config["Izz"].as<double>();
  params.T = config["T"].as<double>();
  params.Qx = config["Qx"].as<double>();
	params.Qy = config["Qy"].as<double>();
	params.Qz = config["Qz"].as<double>();
	params.Qx_dot = config["Qx_dot"].as<double>();
	params.Qy_dot = config["Qy_dot"].as<double>();
  params.Qz_dot = config["Qz_dot"].as<double>();
  params.l = config["l"].as<double>();
  params.b = config["b"].as<double>();
  params.d = config["d"].as<double>();
  params.J_tp = config["J_tp"].as<double>();

  std::vector<double> init_state = config["initial_state"].as<std::vector<double>>();
  for (size_t i = 0; i < init_state.size(); i++) {
    params.initial_state(i) = init_state.at(i);
  }

  std::cout << "--Vehicle parameters--" << '\n'
            << "mass: " << params.m << '\n'
            << "Ixx: " << params.Ixx << '\n'
            << "Iyy: " << params.Iyy << '\n'
            << "Izz: " << params.Izz << '\n'
            << "T: " << params.T  << '\n'
            << "g: " << params.g  << '\n'
            << "Q: " << params.Qx  << '\n'
            << "Qz: " << params.Qz  << '\n'
            << "X_0: " << params.initial_state << '\n'
            << '\n';
  std::vector<SensorParams> sensors;
  std::vector<std::string> id = config["Sensor_prefix"].as<std::vector<std::string>>();
  SensorParams sensor;
  std::vector<double> R, rotation, translation;

  std::cout << "-----Senors-----" << '\n';
  for (size_t i = 0; i < id.size(); i++) {
    sensor.id = id.at(i);
    sensor.topic = config[id.at(i)+"_topic"].as<std::string>();
    sensor.R = MatrixXd::Identity(3, 3);
    R = config[id.at(i)+"_R"].as<std::vector<double>>();
    sensor.R(0,0) = R[0];
    sensor.R(1,1) = R[1];
    sensor.R(2,2) = R[2];
		sensor.R(3,3) = R[3];
		sensor.R(4,4) = R[4];
		sensor.R(5,5) = R[5];

    rotation = config[id.at(i)+"_rotation"].as<std::vector<double>>();
		sensor.w_x = rotation.at(0);
		sensor.w_y = rotation.at(1);
		sensor.w_z = rotation.at(2);

		translation = config[id.at(i)+"_translation"].as<std::vector<double>>();
		sensor.d_x = translation.at(0);
		sensor.d_y = translation.at(1);
		sensor.d_z = translation.at(2);

		sensor.is_odom = config[id.at(i)+"_odom"].as<int>();

    std::cout << "id: " << sensor.id << '\n'
              << "topic: \"" << sensor.topic << "\"\n"
              << "R: \n" << sensor.R << "\n\n";
   sensors.push_back(sensor);
  }

  RosClient obj2(params,sensors, nh_private);
  //obj2.update_dynamics();


  return 0;
}

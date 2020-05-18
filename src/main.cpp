
#include <iostream>
#include "ros/ros.h"
#include "RosClient.hpp"
#include "yaml-cpp/yaml.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "test");
  ros::NodeHandle node_handle;
  std::string config_file;
  node_handle.getParam("config_yaml", config_file);
  
  YAML::Node config = YAML::LoadFile(config_file); //TO-DO path from rosparam

  VehicleParams params;
  params.m = config["mass"].as<double>();
  params.g = config["g"].as<double>();
  params.Ixx = config["Ixx"].as<double>();
  params.Iyy = config["Iyy"].as<double>();
  params.Izz = config["Izz"].as<double>();
  params.T = config["T"].as<double>();
  params.Q = config["Q"].as<double>();
  params.Qz = config["Qz"].as<double>();

  std::cout << "--Vehicle parameters--" << '\n'
            << "mass: " << params.m << '\n'
            << "Ixx: " << params.Ixx << '\n'
            << "Iyy: " << params.Iyy << '\n'
            << "Izz: " << params.Izz << '\n'
            << "T: " << params.T  << '\n'
            << "g: " << params.g  << '\n'
            << "Q: " << params.Q  << '\n'
            << "Qz: " << params.Qz  << '\n'
            << '\n';
  std::vector<SensorParams> sensors;
  std::vector<std::string> id = config["Sensor_prefix"].as<std::vector<std::string>>();
  SensorParams sensor;
  std::vector<double> R;

  std::cout << "-----Senors-----" << '\n';
  for (size_t i = 0; i < id.size(); i++) {
    sensor.id = id.at(i);
    sensor.topic = config[id.at(i)+"_topic"].as<std::string>();
    sensor.R = MatrixXd::Identity(3, 3);
    R = config[id.at(i)+"_R"].as<std::vector<double>>();
    sensor.R(0,0) = R[0,0];
    sensor.R(1,1) = R[1,1];
    sensor.R(2,2) = R[2,2];

    std::cout << "id: " << sensor.id << '\n'
              << "topic: \"" << sensor.topic << "\"\n"
              << "R: \n" << sensor.R << "\n\n";
   sensors.push_back(sensor);
  }

  RosClient obj2(params,sensors);
  obj2.update_dynamics();

  return 0;
}

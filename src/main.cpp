
#include <iostream>
#include "ros/ros.h"
//#include "ekf.hpp"
#include "RosClient.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "test");
  VehicleParams params;
  params.m = 2.3;
  params.g = 9.81;
  params.Ixx = 1;
  params.Iyy = 1;
  params.Izz = 1;
  params.T = 0.04;
  std::cout << "hello world" << std::endl;
  RosClient obj2(params);
  obj2.update_dynamics();

  // const std::string file_name = "test_vector.mat";
  // std::vector<double> v = {1,2,3,4,5,6,7.7};
  // const std::string vec_name = "x";
  // std::ofstream file(file_name);
  // save_vector_as_matrix( vec_name ,v , file );

  return 0;
}

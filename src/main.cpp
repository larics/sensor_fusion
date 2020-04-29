
#include <iostream>
#include "ros/ros.h"
//#include "vehicle_dynamics.hpp"
#include "RosClient.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "test");
  std::cout << "hello world" << std::endl;
  RosClient obj2;
  obj2.update_dynamics();
  //ros::spin();
  return 0;
}

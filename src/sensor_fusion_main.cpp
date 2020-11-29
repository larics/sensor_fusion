#include <iostream>
#include "ros/ros.h"
#include "parse_yaml.h"
#include "ros_client.hpp"
//#include "camera.h"
#include "sensor_client.cpp"
/*
 * Sensor fusion algorithm based on an error state kalman filter
 * This is the main function where we parse data and initialize the
 * filter. Also we set up all ros subscribers and publishers.
 */

int main(int argc, char **argv) {
	ros::init(argc, argv, "Sensor_Fusion");
	ros::NodeHandle node_handle;
	ros::NodeHandle nh_private("~");

	//Next we parse data from the yaml file
	std::string config_file;
	nh_private.getParam("config_yaml", config_file);

	YAML::Node config = YAML::LoadFile(config_file);

	EsEkfParams params = parse_yaml(config_file);


	std::cout << "\n\nModel parameters:\n Q_f \n" << params.model.Q_f << "\n"
						<< "Q_w\n" << params.model.Q_w << "\n\n";
	for (int i = 0; i < params.sensors.size(); ++i) {
		std::cout << "Topic: " << params.sensors.at(i).topic << "\n"
							<< "Id: " <<  params.sensors.at(i).id << "\n"
							<< "R: \n" <<  params.sensors.at(i).cov.R << "\n"
							<< "Rotation:\n" << params.sensors.at(i).rotation_mat << "\n"
							<< "Translation:\n" << params.sensors.at(i).translation
							<< "\n\n";
	}


	//RosClient ros_client(params,nh_private);

	std::cout << "CAMERA to World:\nrotation \n" << params.camera.rotation_mat << "\n"
						<< "translation  " << params.camera.translation.transpose() << "\n"
						<< "is odom " << params.camera.use_camera_imu << std::endl;


	//EsEkf esEkf;
	//Camera camera(params,&esEkf,nh_private);
	SensorClient sensors(params,nh_private);
	sensors.cartographer_callback();
	return 0;
}


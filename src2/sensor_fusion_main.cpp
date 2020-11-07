#include <iostream>
#include "ros/ros.h"
#include "parse_yaml.h"

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
							<< "R: \n" <<  params.sensors.at(i).cov.R
							<< "\n\n";
	}
	


}


#ifndef SENSOR_FUSION_PARSE_YAML_H
#define SENSOR_FUSION_PARSE_YAML_H

#include "yaml-cpp/yaml.h"
#include "structures.h"
#include <iostream>

EsEkfParams parse_yaml(std::string config_file){


	const YAML::Node config = YAML::LoadFile(config_file);

	EsEkfParams params;
	params.model.Q_f(0,0) = config["Q_acc_x"].as<double>();
	params.model.Q_f(1,1) = config["Q_acc_y"].as<double>();
	params.model.Q_f(2,2) = config["Q_acc_z"].as<double>();
	params.model.Q_w(0,0) = config["Q_angular_x"].as<double>();
	params.model.Q_w(1,1) = config["Q_angular_y"].as<double>();
	params.model.Q_w(2,2) = config["Q_angular_z"].as<double>();

	// vector of all sensor parameters
	SensorParams sensor;
	std::vector<double> R, translation, rotation;
	// sensor names and number of sensors
	std::vector<std::string> id = config["Sensor_prefix"].as<std::vector<std::string>>();
	for (size_t i = 0; i < id.size(); i++) {
		sensor.id = id.at(i);
		sensor.topic = config[id.at(i)+"_topic"].as<std::string>();

		/*
		 * Currently the covariance matrix is expected to be
		 * constant and diagonal
		 */
		R = config[id.at(i)+"_R"].as<std::vector<double>>();
		for (int j = 0; j < 3; ++j) {
			sensor.cov.R(j,j) = R[i];
		}

		/*
		 * To get transforamtion from sensor link to base link
		 */
		rotation = config[id.at(i)+"_rotation"].as<std::vector<double>>();
		sensor.rotation_mat = AngleAxisd(rotation.at(0), Vector3d::UnitX())
											 	* AngleAxisd(rotation.at(1), Vector3d::UnitY())
												* AngleAxisd(rotation.at(2), Vector3d::UnitZ());

		translation = config[id.at(i)+"_translation"].as<std::vector<double>>();
		sensor.translation <<  translation.at(0),
													translation.at(1),
													translation.at(2);
		/*
		 * If sensor is odom type, we expect output that defines
		 * relative position in comparison to the old position.
		 * i.e. How much did we move.
		 */
		sensor.is_odom = config[id.at(i)+"_odom"].as<int>();

		params.sensors.push_back(sensor);
	}
	return params;
}


#endif //SENSOR_FUSION_PARSE_YAML_H

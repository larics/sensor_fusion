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
		Matrix<double,3,3> Rot(rotation.data());
		sensor.rotation_mat = Rot;

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

	CameraParams cameraParams;

	std::vector<double> acc = config["camera_acc_cov"].as<std::vector<double>>();
	std::vector<double> lin = config["camera_lin_vel_cov"].as<std::vector<double>>();
	std::vector<double> ang = config["camera_ang_vel_cov"].as<std::vector<double>>();
	std::vector<double> ori = config["camera_orientation_cov"].as<std::vector<double>>();
	std::vector<double> pose = config["camera_pose_cov"].as<std::vector<double>>();
	for (int i = 0; i < 3; ++i) {
		cameraParams.acc_cov.R(i,i) = acc.at(i);
		cameraParams.ang_vel_cov.R(i,i) = ang.at(i);
		cameraParams.lin_vel_cov.R(i,i) = lin.at(i);
		cameraParams.orientation_cov.R(i,i) = ori.at(i);
		cameraParams.pose_cov.R(i,i) = pose.at(i);
	}
	std::vector<double> cam_rotation = config["camera_rotation"].as<std::vector<double>>();
	Matrix<double,3,3> Rot_cam(cam_rotation.data());
	cameraParams.rotation_mat = Rot_cam;
	std::vector<double> cam_translation = config["camera_translation"].as<std::vector<double>>();
	Matrix<double,3,1> D(cam_translation.data());
	cameraParams.translation = D;
	cameraParams.is_odom = config["camera_odom"].as<int>();

	params.camera = cameraParams;

	return params;
}


#endif //SENSOR_FUSION_PARSE_YAML_H

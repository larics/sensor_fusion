#!/bin/bash

export UAV_NAMESPACE=danieli2
export TF_PREFIX=red
export CARTO_NAMESPACE=red
export BAG_NAME=/usr/local/games/compassless_t265_carto_pozyx_2021-09-06-16-21-18.bag
export CONFIGURATION_BASENAME=ouster_local.lua
export CONFIGURATION_DIRECTORY=$(pwd)/custom_config
export CARTO_URDF=$(rospack find uav_ros_general)/urdf/ouster-imu.urdf.xacro
export SF_CONFIG=$(realpath custom_config/sensor_client_paper_config.yaml)
export CARTO_POSE=/$UAV_NAMESPACE/pproc/cartographer/pose

export PURE_LOCALIZATION=false
export PBSTREAM_NAME=

# Starter parameters
export ENABLE_STARTER=true
export USE_FIXED_YAW=true
export FIXED_YAW=1.57

# Define bag topics
export BAG_POINT_TOPIC=/os_cloud_node/points
export BAG_IMU_TOPIC=/red/mavros/imu/data
export BAG_TRANSFORM_TOPIC=/red/pozyx/measured
export BAG_CAMERA_ODOM_TOPIC=/red/camera/odom/sample
export BAG_CARTO_TOPIC=/red/uav/cartographer/pose
export BAG_EKF_TOPIC=/red/es_ekf/odom

# Define cartographer topics
export POINT_TOPIC=/$UAV_NAMESPACE/os_cloud_node/points
export IMU_TOPIC=/$UAV_NAMESPACE/mavros/imu/data
export TRANSFORM_TOPIC=/$UAV_NAMESPACE/pozyx/measured
export CAMERA_ODOM_TOPIC=/$UAV_NAMESPACE/camera/odom/sample
export CARTO_TOPIC=/unused/uav/cartographer/pose
export EKF_TOPIC=/unused/es_ekf_odom
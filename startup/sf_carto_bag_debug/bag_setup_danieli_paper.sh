#!/bin/bash

export UAV_NAMESPACE=danieli2
export BAG_NAME=$HOME/Bags/danieli1_compassless/clanak/measurements_2021-09-06-20-17-17.bag
export CONFIGURATION_BASENAME=ouster_pozyx.lua
export CONFIGURATION_DIRECTORY=$(pwd)/custom_config
export CARTO_URDF=$(rospack find uav_ros_general)/urdf/ouster-imu.urdf.xacro
export SF_CONFIG=$(realpath custom_config/sensor_client_paper_config.yaml)

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

# Define cartographer topics
export POINT_TOPIC=/$UAV_NAMESPACE/os_cloud_node/points
export IMU_TOPIC=/$UAV_NAMESPACE/mavros/imu/data
export TRANSFORM_TOPIC=/$UAV_NAMESPACE/pozyx/measured
export CAMERA_ODOM_TOPIC=/$UAV_NAMESPACE/camera/odom/sample
export CARTO_TOPIC=/$UAV_NAMESPACE/uav/cartographer/pose

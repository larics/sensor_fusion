#!/bin/bash

export UAV_NAMESPACE=danieli2
export BAG_NAME=$HOME/FER/cartographer_experiments/Danieli/Sensor_fusion/sensor_fusion_1.bag
export CONFIGURATION_BASENAME=ouster_pozyx.lua
export CONFIGURATION_DIRECTORY=$(pwd)/custom_config
export CARTO_URDF=$(rospack find uav_ros_general)/urdf/ouster-imu.urdf.xacro

export PURE_LOCALIZATION=false
export PBSTREAM_NAME=

# Starter parameters
export ENABLE_STARTER=true
export USE_FIXED_YAW=true
export FIXED_YAW=1.57

# Define bag topics
export BAG_POINT_TOPIC=/os_cloud_node/points
export BAG_IMU_TOPIC=/danieli2/mavros/imu/data
export BAG_TRANSFORM_TOPIC=/pozyx/measured
export BAG_CAMERA_ODOM_TOPIC=/camera/odom/sample

# Define cartographer topics
export POINT_TOPIC=/$UAV_NAMESPACE/os_cloud_node/points
export IMU_TOPIC=/$UAV_NAMESPACE//mavros/imu/data
export TRANSFORM_TOPIC=/$UAV_NAMESPACE/pozyx/measured
export CAMERA_ODOM_TOPIC=/$UAV_NAMESPACE/camera/odom/sample
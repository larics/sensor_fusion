#!/bin/bash

export UAV_NAMESPACE=red
export TF_PREFIX=red
export CARTO_NAMESPACE=red
export BAG_NAME=/usr/local/games/encore_dataset_2021-07-12-11-57_22_no_camera.bag
export CONFIGURATION_BASENAME=velodyne_local.lua
export CONFIGURATION_DIRECTORY=$(pwd)/custom_config
export CARTO_URDF=custom_config/vlp16-no-ns-imu.urdf.xacro
export SF_CONFIG=$(realpath custom_config/sensor_client_carto_gps_config.yaml)
export SF_CARTO_MAP=eagle/carto_raw

export PURE_LOCALIZATION=false
export PBSTREAM_NAME=

# Starter parameters
export ENABLE_STARTER=true
export STARTER_ODOM_TOPIC=mavros/global_position/local
# Starter transfrom topic is "TRANSFORM_TOPIC"
export USE_FIXED_YAW=false

# Define bag topics
export BAG_POINT_TOPIC=/red/velodyne_points
export BAG_IMU_TOPIC=/red/mavros/imu/data
export BAG_TRANSFORM_TOPIC=
export BAG_CAMERA_ODOM_TOPIC=

# Define cartographer topics
export POINT_TOPIC=/$UAV_NAMESPACE/velodyne_points
export IMU_TOPIC=/$UAV_NAMESPACE/mavros/imu/data
export TRANSFORM_TOPIC=
export CAMERA_ODOM_TOPIC=

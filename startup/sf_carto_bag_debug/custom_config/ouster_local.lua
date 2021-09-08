-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "ouster_pozyx.lua"
namespace = os.getenv("TF_PREFIX")

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame =  namespace.."/map",
  tracking_frame =  namespace.."/base_link",
  published_frame =  namespace.."/base_link",
  odom_frame = namespace.."/odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = true,
  use_odometry = false,
  use_nav_sat = false,
  use_position_sensor = false, -- use POZYX
  position_translation_weight = 1.2, -- 1e0, ----------------- POZYX
  nav_sat_use_predefined_enu_frame = false,
  nav_sat_predefined_enu_frame_lat_deg = 45.813902,
  nav_sat_predefined_enu_frame_lon_deg = 16.038766,
  nav_sat_predefined_enu_frame_alt_m = 168.259294525,
  nav_sat_translation_weight = 1.,
  nav_sat_inverse_covariance_weight = 0.4,
  nav_sat_inverse_covariance_bias = 0,
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 2e-2,
  trajectory_publish_period_sec = 1e0,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

POSE_GRAPH.optimize_every_n_nodes = 0 -- 320 --480
return options

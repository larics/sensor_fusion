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

include "map_builder.lua"
include "trajectory_builder.lua"
namespace = os.getenv("UAV_NAMESPACE")
map_frame = os.getenv("SF_CARTO_MAP")
if  map_frame == nil or map_frame == '' then
  map_frame = namespace.."/map"
end

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame =  map_frame,
  tracking_frame =  namespace.."/base_link",
  published_frame =  namespace.."/base_link",
  odom_frame = namespace.."/odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = true,
  use_odometry = false,
  use_nav_sat = false,
  nav_sat_use_predefined_enu_frame = false,
  nav_sat_predefined_enu_frame_lat_deg = 45.813902,
  nav_sat_predefined_enu_frame_lon_deg = 16.038766,
  nav_sat_predefined_enu_frame_alt_m = 168.259294525,
  nav_sat_translation_weight = 1.,
  nav_sat_inverse_covariance_weight = 0.4,
  nav_sat_inverse_covariance_bias = 1,
  use_position_sensor = false,
  position_translation_weight = 1,
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

TRAJECTORY_BUILDER.collate_fixed_frame = false

TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 5
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 5.5
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight_z = 5.5
TRAJECTORY_BUILDER_3D.min_range = 1
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_3D.submaps.high_resolution = 0.2
TRAJECTORY_BUILDER_3D.submaps.high_resolution_max_range = 40.
TRAJECTORY_BUILDER_3D.submaps.low_resolution = 0.45
TRAJECTORY_BUILDER_3D.submaps.num_range_data = 160.

TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.15

TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter = {
    max_length = 2.,
    min_num_points = 150,
    max_range = 30.,  -- default 50
}

TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter = {
    max_length = 4.,
    min_num_points = 200,
    max_range = 70,
}

MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 3

POSE_GRAPH.optimization_problem.huber_scale =  1 -- Cartographer default 5e2
POSE_GRAPH.optimize_every_n_nodes = 480
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 25  -- Cartographer default 50

POSE_GRAPH.optimization_problem.rotation_weight = 0  -- Cartographer default 3e5
POSE_GRAPH.optimization_problem.acceleration_weight = 1e2  -- Cartographer default 1e3

POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.rotation_weight = 5  -- Cartographer default 1
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.translation_weight = 5  -- Cartographer default 10
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.translation_weight_z = 5

POSE_GRAPH.constraint_builder.global_localization_min_score = 0.66    -- Cartographer default 0.6
POSE_GRAPH.constraint_builder.max_constraint_distance = 15 -- default 15
POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.sampling_ratio = 0.15 -- less > faster algo. Cartographer default 0.3

return options

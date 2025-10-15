include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_footprint",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = true,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 30e-3,
  trajectory_publish_period_sec = 40e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 0.2,
  landmarks_sampling_ratio = 1.,
  use_pose_extrapolator = true,
}
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05
MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.min_range = 0.20
TRAJECTORY_BUILDER_2D.max_range = 10.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 10.15

TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.02
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 280
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.45
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 10.0

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 10,
}

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 100
------------------------------------------------------------------------------------

TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.06
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.4)
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.08

------------------------------------imu------------------------------------------
TRAJECTORY_BUILDER_2D.use_imu_data = true

TRAJECTORY_BUILDER_2D.pose_extrapolator.constant_velocity.imu_gravity_time_constant = 1.  
TRAJECTORY_BUILDER_2D.pose_extrapolator.constant_velocity.pose_queue_duration = 0.15 
TRAJECTORY_BUILDER_2D.pose_extrapolator.imu_based.pose_queue_duration = 0.15
TRAJECTORY_BUILDER_2D.pose_extrapolator.imu_based.gravity_constant = 9.806  
TRAJECTORY_BUILDER_2D.pose_extrapolator.imu_based.pose_translation_weight = 6.0   
TRAJECTORY_BUILDER_2D.pose_extrapolator.imu_based.pose_rotation_weight = 2.5   
TRAJECTORY_BUILDER_2D.pose_extrapolator.imu_based.imu_acceleration_weight = 4.0 
TRAJECTORY_BUILDER_2D.pose_extrapolator.imu_based.imu_rotation_weight = 1.5     
TRAJECTORY_BUILDER_2D.pose_extrapolator.imu_based.odometry_translation_weight = 0.0 
TRAJECTORY_BUILDER_2D.pose_extrapolator.imu_based.odometry_rotation_weight = 0.0 
TRAJECTORY_BUILDER_2D.pose_extrapolator.imu_based.solver_options.use_nonmonotonic_steps = false  
TRAJECTORY_BUILDER_2D.pose_extrapolator.imu_based.solver_options.max_num_iterations = 20  

----------------------------------local---------------------------------------------

TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 2

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true---뺄지 포함할지 고민
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.15
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(25.0)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 0.02
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 0.08


TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = false
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.num_threads = 12
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 30
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 1.0e2
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 4.0e2
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 6.0e1

------------------------------------------------------------------------------------


-------------------------------------global-----------------------------------------
-- Multi
MAP_BUILDER.num_background_threads = 8

-- Constraint Builder
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 8
POSE_GRAPH.constraint_builder.min_score = 0.72
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.85
POSE_GRAPH.constraint_builder.sampling_ratio = 0.015
POSE_GRAPH.global_sampling_ratio = 0.0

POSE_GRAPH.optimization_problem.huber_scale = 5.0 
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1e5
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1e5

-- Optimizer
POSE_GRAPH.optimize_every_n_nodes = 35
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 35
------------------------------------------------------------------------------------

return options
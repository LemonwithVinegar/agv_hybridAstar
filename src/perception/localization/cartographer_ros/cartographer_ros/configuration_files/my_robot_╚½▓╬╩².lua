include "map_builder.lua"		--建图  --pose_graph.lua
include "trajectory_builder.lua"	--定位  --trajectory_builder_2d.lua

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER, 

  map_frame = "map",

  tracking_frame = "base_link",		--如果有imu必须为imu的坐标系

  published_frame = "base_link",

  odom_frame = "odom",
  provide_odom_frame = true, 

--是否将位姿限制为纯2D pose
  publish_frame_projected_to_2d = false,

--是否使用位姿外推器
  use_pose_extrapolator = false,  --不要使用，原因未知（没有imu等数据？？？）

--是否发布追踪的位姿，需要发
  publish_tracked_pose = true,	

--是否使用里程计(odom)、导航信息(GPS)、路标（需要发布相应话题给slam）
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,

--选择激光输入的话题为points2
  num_laser_scans = 0,	
  num_multi_echo_laser_scans = 0,
  num_point_clouds = 1,	

--将一帧激光拆分成的几次发出，以减少运动带来的雷达数据的畸变
  num_subdivisions_per_laser_scan = 1,

--【时间参数】
--使用tf2查找变换的超时时间
  lookup_transform_timeout_sec = 0.2,
--子图的发布间隔，s
  submap_publish_period_sec = 0.3,
--发布位姿的时间间隔
  pose_publish_period_sec = 5e-3,
--发布轨迹标记的间隔
  trajectory_publish_period_sec = 30e-3,

--【采样参数】
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,		--如果odom数据不准，可以减少权重为0.3之类
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

--后端线程数
MAP_BUILDER.num_background_threads = 6		
--使用2D-slam
MAP_BUILDER.use_trajectory_builder_2d = true 
--不使用imu
TRAJECTORY_BUILDER_2D.use_imu_data = false
--必须配置！！！否则建图垃圾
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
--多少个点云作为运动补偿
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1


--[local SLAM]
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1.)	--只有当scan的平移、旋转超过阈值时才会被加入到submap
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.1	--太大直线建图都是斜的，太小转弯之后是斜的
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90	--一个子图多少帧数据

TRAJECTORY_BUILDER_2D.min_z = -0.2		--scan 3D to 2D
TRAJECTORY_BUILDER_2D.max_z = 0.5

TRAJECTORY_BUILDER_2D.min_range = 0.2
TRAJECTORY_BUILDER_2D.max_range = 12
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 12	--如果不在min_range和max_range之间，则默认给这个数值


--[global SLAM]
POSE_GRAPH.optimize_every_n_nodes = 0	 	--多少个节点开始优化，=0，则关闭回环，默认90，	--定位时减少
POSE_GRAPH.global_sampling_ratio = 0.003	--全局采样率					--定位时减少
POSE_GRAPH.max_num_final_iterations = 200	--全局优化

POSE_GRAPH.constraint_builder.min_score = 0.65	--扫描匹配分数的阈值，低于该阈值时不考虑匹配 
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3	--约束采样率				--定位时减少

return options









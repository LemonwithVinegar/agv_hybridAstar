#! /bin/zsh

path=`pwd`
rosservice call /finish_trajectory 0
rosservice call /write_state "{filename: '${path}/data/x.pbstream'}" 

# 1-kill slam
rosnode   kill   /cartographer_node    /cartographer_occupancy_grid_node  
#rosnode   kill   $(rosnode list | grep "/cartographer_node")
#rosnode   kill   $(rosnode list | grep "/cartographer_occupancy_grid_node")

# 2-reset slam-flag 
#rosparam  get  /Is_slam_start
rosparam  set  /Is_slam_start  0 
#rosparam  get  /Is_slam_start 

#rosparam  get  /slam_origin_get_once
rosparam  set  /slam_origin_get_once  false 
#rosparam  get  /slam_origin_get_once

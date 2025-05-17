# rrt_star_global_planner

经过优化在障碍物较少的区域能快速规划出路径，目标点在狭窄又弯曲的地方需要很长时间（测试达到16秒左右）才能规划出路径。
对规划耗时影响较大的是目标点，当目标点在狭窄弯曲的区域时耗时较长。


测试插件功能
roslaunch mbot_gazebo view_mbot_with_laser_gazebo.launch
roslaunch mbot_navigation exploring_slam_demo.launch


roslaunch mbot_navigation nav_cloister_demo.launch



rostopic pub -1 /cur_pose geometry_msgs/Pose2D "{x: 1.0, y: 2.0, theta: 0.0}"
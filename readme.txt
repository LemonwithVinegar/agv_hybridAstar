这份代码相对于其他代码规划部分和路径发布部分有巨大不同，其它有些部分有小修改，使用方法和其它一样
1、新增Hybrid_A_Star，环境配置参考Hybrid_A_Star中的readme.md，ros版本替换为对应的版本即可
2、custom_msgs中因为需要新加了一些
3、规划中新加了hyastar_task_pub功能包，道路属性通过GetRoadAtr()函数从config/road_attributes.txt中读取，txt文件中的速度只能设置为0，要设置车的速度在每个任务中自己设置。
任务主要是前进、后退、停车，根据当前路段的s，以及是前进还是后退在hyastar_task_pub功能包中写代码实现,每个任务的类型对应custom_msgs/srv/Task.srv中的数字，主要使用了4和8，任务执行函数在task_server.cpp为Back_car_mission（包括了前进和后退任务）和stop_run_car，可以根据需要自己实现
4、Hybrid_A_Star文件夹中cpp主要功能流程：
根据车当前位置和目标点规划路径，然后对路径分段，然后发布第一段路径到road_lane.cpp那一部分，
处理获得当前的s，任务那边根据当前路径段的s执行任务，每一段路径执行完最终停下后发布一个停车状态标志位，再发布第二段路径。goal_pose_pub.cpp从data/points.txt中读取目标点（x y theta）发布。如果要更换地图在Hybrid_A_Star/maps中，同时gnss2slam函数中下面几个参数需要改（gnss原点与slam原点的坐标系变换关系 （base_link in map）），有两处

        transformStamped.transform.translation.x = 1.9;
        transformStamped.transform.translation.y = -5.3;
        transformStamped.transform.translation.z = 0.0;

        transformStamped.transform.rotation.x = 0;  // xyz w
        transformStamped.transform.rotation.y = 0;
        transformStamped.transform.rotation.z = -0.982;
        transformStamped.transform.rotation.w = 0.191;	
run_hybrid_a_star.launch中的args="1.9 -5.3 0  0 0 -0.982 0.191 map slam_map 100"/>参数也要改
5、其余在原有代码基础上开发的部分都有注释
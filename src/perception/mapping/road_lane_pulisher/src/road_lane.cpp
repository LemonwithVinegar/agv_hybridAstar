#include "road_lane.h"

double start_s = 0.0;

RoadLane::RoadLane()
{
    map_srv = nh.advertiseService("/map_switch_server", &RoadLane::map_switch,this); //发布地图id 切换服务

    road_lane_pub = nh.advertise<custom_msgs::LaneLineArray>("/road_lane", 1, true);
    cur_pose_sd_pub = nh.advertise<custom_msgs::CurPose>("/cur_pose_all", 1, true);
    //velocity_pub = nh.advertise<geometry_msgs::Vector3>("/map_velocity", 1,true);
    //turn_light_num = 1;
    IsInit = true;

    std::string cur_pose_topic;
    nh.param<std::string>("/node_road_lane/cur_pose_topic", cur_pose_topic,"/cur_pose");
    cur_pose_sub = nh.subscribe(cur_pose_topic,1,&RoadLane::CurPoseRecvd,this);     ///cur_pose才会触发地图发布

    searched_points_sub = nh.subscribe("/run_hybrid_astar/searched_points", 1, &RoadLane::onLaneLineArrayMsgRecvd,this);
    pub_s_dir = nh.advertise<custom_msgs::CarDirection>("/car_direction", 1, true);
}

bool RoadLane::map_switch(custom_msgs::Map_Switch::Request &req,
         custom_msgs::Map_Switch::Response &res){
    GetRoadLane(req.id);
    res.isSuccess = true;
    return true;
}

//转换矩阵获取：ros==>Eigen的接口 
bool RoadLane::TransformToMatrix(const geometry_msgs::TransformStamped& trans, Eigen::Matrix4f& transform_matrix) {  

    double roll, pitch, yaw;
	tf::StampedTransform transform;
	//
	transform.setOrigin( tf::Vector3(trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z) );
	transform.setRotation( tf::Quaternion(trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w) );
    //平移 
    Eigen::Translation3f tl_btol(transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ());
    //姿态
    tf::Matrix3x3( transform.getRotation() ).getEulerYPR(yaw, pitch, roll); //ros中的角

    Eigen::AngleAxisf rot_x_btol( roll, Eigen::Vector3f::UnitX() );     //转为Eigen中的旋转向量， 以x轴旋转roll弧度
    Eigen::AngleAxisf rot_y_btol( pitch, Eigen::Vector3f::UnitY() ); 
    Eigen::AngleAxisf rot_z_btol( yaw, Eigen::Vector3f::UnitZ() ); 

    //TF变换转换到变换矩阵 
    transform_matrix = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();  //右乘

    return true; 
}


void RoadLane::slam2gnssmap(const geometry_msgs::Pose::ConstPtr &msg)  	 
{
    geometry_msgs::TransformStamped  slamTrans; 
    Eigen::Matrix4f Tso_car, Tgo_car;
    Eigen::Matrix3f rotationMatrix;
    Eigen::Matrix4f Tgo_so; 
    Eigen::Vector3f euler_angles;

    if (1) {    // 已知gnss原点与slam原点的坐标系变换关系 （base_link in map）
        transformStamped.transform.translation.x = 1.9;
        transformStamped.transform.translation.y = -5.3;
        transformStamped.transform.translation.z = 0.0;

        transformStamped.transform.rotation.x = 0;  // xyz w
        transformStamped.transform.rotation.y = 0;
        transformStamped.transform.rotation.z = -0.982;
        transformStamped.transform.rotation.w = 0.191;	
    	}
    TransformToMatrix(transformStamped, Tgo_so); 

    slamTrans.transform.translation.x = msg->position.x;
    slamTrans.transform.translation.y = msg->position.y;
    slamTrans.transform.translation.z = msg->position.z;
    slamTrans.transform.rotation = msg->orientation;
    
    //convert   position  and angle
    TransformToMatrix(slamTrans, Tso_car); 
    Tgo_car = Tgo_so * Tso_car; 
    rotationMatrix = Tgo_car.block<3, 3>(0, 0);
    euler_angles = rotationMatrix.eulerAngles( 2,1,0 );	// ZYX顺序，即yaw pitch roll顺序  取出z轴偏航角即可
    // std::cout << "原始 euler_angles(0): " << (euler_angles(0) * 180 / M_PI) << std::endl;
    // std::cout << "原始 euler_angles(1): " << (euler_angles(1) * 180 / M_PI) << std::endl;
    // std::cout << "原始 euler_angles(2): " << (euler_angles(2) * 180 / M_PI) << std::endl;

    /*****************get slam pose on map*******************************/
    euler_angles(0) = euler_angles(2)!=0 ? (M_PI - euler_angles(0)) : (2*M_PI - euler_angles(0));   //heading
    double aftYaw = (euler_angles(0) >= 0 && euler_angles(0) < M_PI/2) ?  (M_PI/2 - euler_angles(0)) : (2.5*M_PI - euler_angles(0));  //cur_pose

    // 准换后的2D位姿 
    slam_pose.x = Tgo_car(0, 3); 
    slam_pose.y = Tgo_car(1, 3); 
    slam_pose.theta = aftYaw * 180 / M_PI;  
}

void RoadLane::GetRoadLane(const int &lane_id)
{
    int last_id = lane_id;
    refer_road_lane.x.clear();
    refer_road_lane.y.clear();
    refer_road_lane.s.clear();
    left_road_lane.x.clear();
    left_road_lane.y.clear();
    right_road_lane.x.clear();
    right_road_lane.y.clear();
    custom_msgs::LaneLineArray road_lane;
    double sum_refer_s = 0;
    cur_lane = select_Lane_by_id(lane_id);
    int cur_refer_line_size = cur_lane.ReferencePoints_current_line.size();
    // refer_road_lane.x.push_back(cur_lane.ReferencePoints_current_line[0].x);
    // refer_road_lane.y.push_back(cur_lane.ReferencePoints_current_line[0].y);

    for(int i = 0; i < cur_refer_line_size; i++)    //参考点 xy
    {
        refer_road_lane.x.push_back(cur_lane.ReferencePoints_current_line[i].x);
        refer_road_lane.y.push_back(cur_lane.ReferencePoints_current_line[i].y);
    }

    for(int i = 0; i < cur_refer_line_size; i++)     //参考点 sd
    {
        sum_refer_s = getFrenet2(cur_lane.ReferencePoints_current_line[i].x,cur_lane.ReferencePoints_current_line[i].y,
                                    refer_road_lane.x,refer_road_lane.y,s_start[cur_lane.current_lane_id])[0];
        refer_road_lane.s.push_back(sum_refer_s);
    }

    refer_road_lane.id = 0;
    refer_road_lane.type = 0;
    refer_road_lane.current_lane_num = cur_lane.current_lane_id;
    int cur_left_line_size = cur_lane.LanePoints_left_current_line.size();
    for(int i = 0; i < cur_left_line_size; i++)     //左点
    {
        left_road_lane.x.push_back(cur_lane.LanePoints_left_current_line[i].x);
        left_road_lane.y.push_back(cur_lane.LanePoints_left_current_line[i].y);
    }
    left_road_lane.id = 1;
    left_road_lane.type = 1;
    int cur_right_line_size = cur_lane.LanePoints_right_current_line.size();
    for(int i = 0; i < cur_right_line_size; i++)    //右点
    {
        right_road_lane.x.push_back(cur_lane.LanePoints_right_current_line[i].x);
        right_road_lane.y.push_back(cur_lane.LanePoints_right_current_line[i].y);
    }
    right_road_lane.id = 2;
    right_road_lane.type = 1;
    road_lane.lines.push_back(refer_road_lane);
    road_lane.lines.push_back(left_road_lane);
    road_lane.lines.push_back(right_road_lane);
    road_lane_pub.publish(road_lane);           //道路发布

    if(abs(cur_lane.current_lane_id) <= ID_SUM && abs(cur_lane.current_lane_id)>0){
         cout<<"____________road_lane publish OK!!!"<<endl;
         std::cout << "________Current_lane_id: " << cur_lane.current_lane_id <<std::endl;
         std::cout << "___________Next_lane_id: " << cur_lane.next_lane_id <<std::endl;
    }else{

         ROS_WARN("road_lane publish falied! Please check map_origin set is right? or Gnss_localization is right?");
    }
}

//初始化两个s数组的大小
void RoadLane::init_s_size(int laneCnt){
    s_start.resize(laneCnt+10);
    s_end.resize(laneCnt+10);
}

void RoadLane::cal_s_value(int laneCnt){
    s_start[0] = 0;
    s_end[0] = 0;
    s_start[1] = 0;
    for(int i = 1; i <= laneCnt; i++){
        LanePoints.clear();
        GetLaneBEP(i, LanePoints);
        if(LanePoints.size() == 0){
            s_end[i] = s_start[i];
            s_start[i+1] = s_end[i];
            continue;
        }

        std::vector<double> map_x;
        std::vector<double> map_y;
        for(int i = 0; i < LanePoints.size();i++){
    
            map_x.push_back(LanePoints[i].x);
            map_y.push_back(LanePoints[i].y);
        }
        s_end[i] = getFrenet2(map_x[map_x.size()-1],map_y[map_y.size()-1],map_x,map_y,s_start[i])[0];
        s_start[i+1] = s_end[i];

    }
}

void RoadLane::IsLaneInit(const geometry_msgs::Pose2D &msg)
{
    if(IsInit)
    {
        //初始化每一个s的数组
        int laneCnt = GetLaneCnt();
        init_s_size(laneCnt);
        cal_s_value(laneCnt);
        
        int cur_id = Get_Lane_id_by_Coor(msg.x, msg.y); //根据xy判断哪段map
        IsInit = false;
        GetRoadLane(cur_id);    //根据id发布
    }
}


void RoadLane::CurPoseRecvd(const geometry_msgs::Pose2D &msg)
{
    /*
    IsLaneInit(msg);    //通过当前x，y判断该加载哪段地图， 并完成该段地图加载
    
    std::vector<double> start_frenet = getFrenet2(cur_lane.current_lane_start_point.x,cur_lane.current_lane_start_point.y,refer_road_lane.x,refer_road_lane.y,s_start[cur_lane.current_lane_id]);
    std::vector<double> end_frenet = getFrenet2(cur_lane.current_lane_end_point.x, cur_lane.current_lane_end_point.y,refer_road_lane.x,refer_road_lane.y,s_start[cur_lane.current_lane_id]);
    std::vector<double> cur_frenet = getFrenet2(msg.x,msg.y,refer_road_lane.x,refer_road_lane.y,s_start[cur_lane.current_lane_id]);
    
    // std::cout << "______________________" << std::endl;
    // std::cout << cur_frenet[0] << " " << end_frenet[0] << std::endl;
    if((cur_frenet[0]+1 >= end_frenet[0]) && IsInit==false)
    {
        GetRoadLane(cur_lane.next_lane_id); 
        start_frenet = getFrenet2(cur_lane.current_lane_start_point.x,cur_lane.current_lane_start_point.y,refer_road_lane.x,refer_road_lane.y,s_start[cur_lane.current_lane_id]);
        end_frenet = getFrenet2(cur_lane.current_lane_end_point.x, cur_lane.current_lane_end_point.y,refer_road_lane.x,refer_road_lane.y,s_start[cur_lane.current_lane_id]); 
        cur_frenet = getFrenet2(msg.x,msg.y,refer_road_lane.x,refer_road_lane.y,s_start[cur_lane.current_lane_id]);
    }  
    */

   if (refer_road_lane.x.size()==0)
        return;

    std::vector<double> cur_frenet = getFrenet2(msg.x,msg.y,
                                                refer_road_lane.x,refer_road_lane.y,
                                                start_s);     // id


    custom_msgs::CurPose cur_pose_all;
    cur_pose_all.x = msg.x;
    cur_pose_all.y = msg.y;
    cur_pose_all.theta = msg.theta;
    cur_pose_all.s = cur_frenet[0];
    cur_pose_all.d = cur_frenet[1];

    cur_pose_sd_pub.publish(cur_pose_all);  // 发布当前的s、d
    
}

void RoadLane::onLaneLineArrayMsgRecvd(const geometry_msgs::PoseArray::ConstPtr& msg) {

    int id = msg->poses[1].position.z;        // 段id
    int dir = msg->poses[1].orientation.x;    // 车辆行驶方向
    int LastSegment = msg->poses[1].orientation.y;    // 判断该段路径是否为最后一段，LastSegment=1是，=0不是

    refer_road_lane.x.clear();
    refer_road_lane.y.clear();
    refer_road_lane.s.clear();

    for (const auto& pose : msg->poses) {
        slam2gnssmap(geometry_msgs::Pose::ConstPtr(&pose, [](const geometry_msgs::Pose*){}));
        refer_points.x.push_back(slam_pose.x);
        refer_points.y.push_back(slam_pose.y);
        // std::cout << "slam_pose.x:"<<slam_pose.x << " " << "slam_pose.y:" <<slam_pose.y<< std::endl;
    }
    
    for (size_t i = 0; i < refer_points.x.size(); i++) {
        refer_road_lane.x.push_back(refer_points.x[i]);
        refer_road_lane.y.push_back(refer_points.y[i]);
    }

    double sum_refer_s = 0.0;
    for(int i = 0; i < refer_points.x.size(); i++)     //参考点 sd
    {
        sum_refer_s = getFrenet2(refer_road_lane.x[i], refer_road_lane.y[i],
                                    refer_road_lane.x, refer_road_lane.y, 
                                    id)[0];

        refer_road_lane.s.push_back(sum_refer_s);
        // std::cout << "s :\n" << sum_refer_s << std::endl;
    }
    start_s = refer_road_lane.s.at(0);
    custom_msgs::LaneLineArray road_lane;
    road_lane.lines.push_back(refer_road_lane);
    road_lane_pub.publish(road_lane);           //道路发布

    custom_msgs::CarDirection  val;     // 发布每段的末尾s值 和 车辆行驶方向
    val.start_s = refer_road_lane.s[0];
    val.direction = dir;
    val.end_s = sum_refer_s;
    val.lastSegment = LastSegment;
    val.path_id = id;
    pub_s_dir.publish(val);        
}

int RoadLane::exec()
{
    ros::spin();
    return 0;
}


  
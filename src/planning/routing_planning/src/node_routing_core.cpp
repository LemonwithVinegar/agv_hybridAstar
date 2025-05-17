#include "node_routing_core.h"

Node_Routing_Core::Node_Routing_Core()
{
    change_lane_info.is_change_lane = false;
    change_lane_info.offset_postion = 0;               
    change_lane_info.first_s = 0;  

    abe_stop.enable = false;
    speed_task.enable = false;
    backcar_task.enable = false;



//订阅话题
    sub_laneline_arry = nh.subscribe("road_lane", 1, &Node_Routing_Core::onLaneLineArrayMsgRecvd,this);
    sub_current_pose = nh.subscribe("cur_pose_all",1,&Node_Routing_Core::onCurrentPoseMsgRecvd,this);   
    sub_navi_data = nh.subscribe("navi_msg", 1, &Node_Routing_Core::onNaviDataRecvd, this);
    sub_road_speed = nh.subscribe("/road_attri_msg",1,&Node_Routing_Core::onRoadSpeedMsgRecvd,this);
    vehicle_stat_sub  = nh.subscribe("/VehicleStat", 1, &Node_Routing_Core::onVehicleStatRecvd, this);

//service服务端
    routing_service = nh.advertiseService("/routing_service", &Node_Routing_Core::onRoutingHandleFunction,this);
    control_service = nh.advertiseService("/control_service", &Node_Routing_Core::onControlHandleFunction,this);

//发布话题：通知topic master话题管理器，该节点将要发布话题的名称，数据类型，缓区大小，未来被接受的数据是否被自动抛弃。
    pub_request = nh.advertise<custom_msgs::Request>("/Request",1,true);   //false
    pub_path = nh.advertise<custom_msgs::Path>("raw_path",1);
    pub_way_point_vis = nh.advertise<visualization_msgs::Marker>("waypoints_ref",5,true);
    pub_path_vis = nh.advertise<nav_msgs::Path>("path_vis",1);
    paths.x_ref.resize(30);
    paths.y_ref.resize(30);
     
    path_vis.header.frame_id = "center_back";   //center_back  base_link  imu

    //marker.ns = "path_ref";
    // marker.id = 1;
    // marker.action = visualization_msgs::Marker::ADD;
    // marker.type = visualization_msgs::Marker::LINE_STRIP;
    // marker.scale.x = 0.05;    //线条的粗细
    // marker.color.a = 1;
    // marker.color.r = 0;
    // marker.color.g = 0;
    // marker.color.b = 1;
    // marker.header.frame_id = "map";
    // marker.lifetime = ros::Duration(0);
}

void Node_Routing_Core::exec()
{
    // marker.points.resize(paths.map_waypoints_x.size());
    // for (size_t i = 0; i<marker.points.size(); ++i)
    // {
    //     geometry_msgs::Point &point = marker.points[i];
    //     point.x = paths.map_waypoints_x[i];
    //     point.y = paths.map_waypoints_y[i];
    //     point.z = 0;
    // }
    // pub_way_point_vis.publish(marker);
    ros::spin();
}




//接收车道线信息    
void Node_Routing_Core::onLaneLineArrayMsgRecvd(const custom_msgs::LaneLineArray::ConstPtr &msg)  
{
    line_ref.points.clear();
    Point point_ref;
    for (int j = 0; j < ( (msg->lines).at(0).x.size() ); j++)      //lines[0]为参考路径点信息  
    {
        point_ref.x = ( (msg->lines).at(0) ).x.at(j);
        point_ref.y = ( (msg->lines).at(0) ).y.at(j);
        point_ref.s = ( (msg->lines).at(0) ).s.at(j);             //s[0]为该路段起始s

        line_ref.points.push_back(point_ref);
    }    
        line_ref.line_nums = ( (msg->lines).at(0) ).current_lane_num;  

    return;
}



//接收主车当前位姿   
void Node_Routing_Core::onCurrentPoseMsgRecvd(const custom_msgs::CurPose::ConstPtr &msg)  
{
    
    current_pose.yaw = deg2rad(msg->theta);                              //航向角转为弧度制
    // current_pose.x = msg->x + translation_para * cos(current_pose.yaw);  
    // current_pose.y = msg->y + translation_para * sin(current_pose.yaw);

    if(is_Parking)
    {
        current_pose.x = msg->x - trans_para_park * cos(current_pose.yaw);
        current_pose.y = msg->y - trans_para_park * sin(current_pose.yaw);
        //cout<<"parking"<<endl;
    }
    else
    {
        current_pose.x = msg->x - trans_para_back * cos(current_pose.yaw);
        current_pose.y = msg->y - trans_para_back * sin(current_pose.yaw);
        // current_pose.x = msg->x + trans_para_forward * cos(current_pose.yaw);
        // current_pose.y = msg->y + trans_para_forward * sin(current_pose.yaw);
        //cout<<"go_ahead"<<endl;
    }

    vector<double> map_waypoints_x_temp;    
    vector<double> map_waypoints_y_temp;
    for (int j = 0; j < ( line_ref.points.size() ); j++)
    {
        map_waypoints_x_temp.push_back( ( line_ref.points.at(j).x ) ) ;
        map_waypoints_y_temp.push_back( ( line_ref.points.at(j).y ) ) ;
    } 
     vector<double> current_sd_para = getFrenet2(current_pose.x, current_pose.y, map_waypoints_x_temp, map_waypoints_y_temp,line_ref.points.at(0).s);
     current_pose.s = current_sd_para[0];
     current_pose.d = current_sd_para[1];

    // current_pose_temp.yaw = deg2rad(msg->theta);
    // current_pose_temp.x = msg->x + 0 * cos(current_pose_temp.yaw);
    // current_pose_temp.y = msg->y + 0 * sin(current_pose_temp.yaw);

    paths.generate_path(current_pose, line_ref, navi_data, change_lane_info);     //执行路径规划模块   

    toPath(path,current_pose);         //传出规划路径，传出path      current_pose_temp
/*
    //规划直行，不跟随地图  旭硕出库场景
    if(change_lane_info.is_change_lane == true && change_lane_info.offset_postion == 0)
    {
        for(int i = 0; i < 30; ++i){
        path.x_ref[i] = 0;
        path.y_ref[i] = i*0.5;
        }
    }
*/
    pub_path.publish(path);                 //发布路径
    
    toPathVis(path,path_vis);               //传出规划路径到RVIZ消息
    pub_path_vis.publish(path_vis);         //发布路径到RVIZ


    ros::spinOnce();
    return;
}


//接收主车当前速度
void Node_Routing_Core::onNaviDataRecvd(const custom_msgs::NaviData::ConstPtr &msg)  
{
    navi_data.velocity = msg->speed2d;  //主车速度,单位km/h
    return;
}


//传出规划路径（转换为车辆坐标系下）,custom_msgs::Path &path_msg为传出参数
void Node_Routing_Core::toPath(custom_msgs::Path &path_msg,CurrentPose &current_pose_temp)  
{
    path_msg.x_ref.resize(paths.x_ref.size());
    path_msg.y_ref.resize(paths.y_ref.size());
    for (int i = 0; i < paths.x_ref.size(); i++)
    {
        double shift_x = paths.x_ref[i] - current_pose_temp.x;
        double shift_y = paths.y_ref[i] - current_pose_temp.y;
        
        //将路径点转换到车辆坐标系（后轮中心）
        path_msg.x_ref[i]=shift_x * sin(current_pose_temp.yaw) - shift_y * cos(current_pose_temp.yaw);
        path_msg.y_ref[i]=shift_y * sin(current_pose_temp.yaw) + shift_x * cos(current_pose_temp.yaw);
    }
    return;
}

void Node_Routing_Core::toPathVis(const custom_msgs::Path &src, nav_msgs::Path &dst)
{
    dst.poses.resize(src.x_ref.size());
    for (size_t i = 0; i<src.x_ref.size(); ++i)
    {
        geometry_msgs::Point &p = dst.poses[i].pose.position;
        p.x = src.x_ref[i];
        p.y = src.y_ref[i];
        p.z = 0;
    }
    return;
}

 //速度接收与处理  request下发
void Node_Routing_Core::onRoadSpeedMsgRecvd(const custom_msgs::RoadAttri::ConstPtr &msg)     
{

    double spe_differ;   
    double Shortest_dis = 100;
    bool is_have_abe = false;
    bool is_have_stop = false;
    
    for(iter=AEB_list.begin(); iter!=AEB_list.end(); ++iter){
        if(iter->second.enable)
        {
            is_have_abe = true;
            request.reques_type = custom_msgs::Request::AEB_ENABLE;
            Shortest_dis = iter->second.info < Shortest_dis? iter->second.info : Shortest_dis;
            request.aeb_distance = Shortest_dis;
            pub_request.publish(request);
            ROS_WARN_STREAM_THROTTLE(1,iter->first << " triggered AEB !!!!!!");
        } 
    }     
    if(is_have_abe) return;
    
    for(iter=STOP_list.begin(); iter!=STOP_list.end(); ++iter){
        if(iter->second.enable)
        {
            is_have_stop = true;
            request.reques_type = custom_msgs::Request::STOP_ENABLE;
            Shortest_dis = iter->second.info < Shortest_dis? iter->second.info : Shortest_dis;
            request.stop_distance = Shortest_dis;
            pub_request.publish(request);
            
            ROS_WARN_STREAM_THROTTLE(1,iter->first << " triggered STOP !!!!!!");
        } 
    }
    if(is_have_stop) return;

    Expedspeed = speed_task.enable? speed_task.info : msg->velocity;
    spe_differ = Expedspeed - navi_data.velocity;      
    if(fabs(spe_differ) <2 )  request.run_speed = Expedspeed;
    else request.run_speed = navi_data.velocity + spe_differ*speed_proportion; 
    request.run_speed = Expedspeed;
    request.reques_type = backcar_task.enable? custom_msgs::Request::BACK_ENABLE : custom_msgs::Request::FORWARD_ENABLE;    
    pub_request.publish(request);

    return;
}




bool Node_Routing_Core::onRoutingHandleFunction(custom_msgs::Route::Request &req,custom_msgs::Route::Response &res)   //routing_service处理函数
{

    change_lane_info.is_change_lane = req.enable;                 //变道标志位
    change_lane_info.offset_postion = req.target_d;               //横向偏移目标位置
    change_lane_info.first_s = req.target_s;                  
	res.isSuccess =	true;

	return	true;
}


bool Node_Routing_Core::onControlHandleFunction(custom_msgs::Control::Request &req,custom_msgs::Control::Response &res)   //control_service处理函数
{

    switch( req.type )
     {
        case 1:     //Is_stop
          abe_stop.enable = req.enable;

        //   if (abe_stop.enable)
        //     std::cout << "................................" << std::endl;
        //     else 
        //      std::cout << "////////////////////////////////" << std::endl;

          abe_stop.info = req.info;
          STOP_list[req.source] = abe_stop;     //无则添加，有则修改

          if( req.source == "ALL"){
              for(iter=STOP_list.begin(); iter!=STOP_list.end(); ++iter)
              iter->second.enable = false;
              ROS_INFO("ALL STOP CANCELED!!!!!!!!!!");
          }
          break;

        case 2:     //Is_aeb
          abe_stop.enable = req.enable;
          abe_stop.info = req.info;
          AEB_list[req.source] = abe_stop;     //无则添加，有则修改
          break;

        case 3:   //Is_back
          backcar_task.enable = req.enable;
          break;

        case 4:   //Is_switch_speed
          speed_task.enable = req.enable;
          speed_task.info = req.info;
          break;
        default  : ROS_INFO("Control.srv's type is error!!!!!!");
     }
 
    res.isSuccess =	true;



	return	true;
}




void Node_Routing_Core::onVehicleStatRecvd(const custom_msgs::VehicleStat::ConstPtr &msg){
    if(msg->GearShiftPositon == -1)
        is_Parking = true;
    else
        is_Parking = false;
}
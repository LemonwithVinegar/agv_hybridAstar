#include "task_server.h"
#include <iostream>
using namespace std;

#include <unistd.h>

volatile int exec_time; //等待启动任务启动一段时间之后才可以进行关闭slam的任务

/**
 * @brief 任务触发回调
 * @param 
 */
bool task_server::onTaskCallRecvd(custom_msgs::Task::Request &req,
                                  custom_msgs::Task::Response &res){

    ROS_WARN_STREAM("I recv a mission : "<<(int)req.task_type);
 
    switch (req.task_type){
        case custom_msgs::Task::Request::Is_Light_mission:{
            std::thread thread(&task_server::Light_mission,this, req.s_start, req.s_end, req.info);
            thread.detach();
            break;
        }   
        case custom_msgs::Task::Request::Is_traffic_light:{
            std::thread thread(&task_server::Pass_traffic_light_mission,this, req.s_start, req.s_end, req.info);
            thread.detach();
            break;
        }
        case custom_msgs::Task::Request::Is_Avoid_obstacle:{
            std::thread thread(&task_server::Avoid_obstacle_mission,this, req.s_start, req.s_end, req.info);
            thread.detach();
            break;
        }
        case custom_msgs::Task::Request::Is_Back_car:{
            std::thread thread(&task_server::Back_car_mission,this, req.s_start, req.s_end, req.info ,req.info_2);
            thread.detach();
            break;
        }
        case custom_msgs::Task::Request::Is_Pass_gate:{
            std::thread thread(&task_server::Pass_gate_mission,this, req.s_start, req.s_end, req.info);
            thread.detach();
            break;
        }
        case custom_msgs::Task::Request::Is_change_map:{
            std::thread thread(&task_server::Change_map,this, req.s_start, req.s_end, req.info);
            thread.detach();
            break;
        } 
        case custom_msgs::Task::Request::Is_slam_change_map:{       //slam地图跳段  暂时未用到 
            std::thread thread(&task_server::slam_changeMap,this, req.s_start, req.s_end, req.info);
            thread.detach();
            break;
        }
        case custom_msgs::Task::Request::Is_stop_run_car:{
            std::thread thread(&task_server::stop_run_car,this, req.s_start, req.s_end, req.info);
            thread.detach();
            break;
        } 
        case custom_msgs::Task::Request::Is_find_obj:{
            std::thread thread(&task_server::find_obj,this, req.s_start, req.s_end, req.info);
            thread.detach();
            break;
        }
        case custom_msgs::Task::Request::Is_start_slam:{       //gnss+slam切换
            std::thread thread(&task_server::start_slam_mission,this, req.s_start, req.s_end, req.info);
            thread.detach();
            break;
        }
        case custom_msgs::Task::Request::Is_close_slam:{       //gnss+slam切换
            std::thread thread(&task_server::close_slam_mission,this, req.s_start, req.s_end, req.info);
            thread.detach();
            break;
        }              
        default:
            break;
    }
    res.isSuccess = true;
    return true;
}

//*******************************************************消息回调**************************************************************
/**
 * @brief 接收到定位消息的回调函数
 * @param 
 */
void task_server::onNaviMsgRecvd(const custom_msgs::NaviData::ConstPtr &msg){
    ros_navi_data = *msg;
}

/**
 * @brief 接收到位置消息的回调函数
 * @param 
 */
void task_server::onCurPoseSDRecvd(const custom_msgs::CurPose::ConstPtr &msg){
    mtx_pose.lock();
    ros_cur_pose = *msg;
    mtx_pose.unlock();
}

/**
 * @brief 接收到目标消息的回调函数
 * @param 
 */
void task_server::onTargetRecvd(const custom_msgs::ObjectArray::ConstPtr &msg){
    mtx_object.lock();
    ros_object_arry = *msg;
    mtx_object.unlock();
}

/**
 * @brief 接收到道路属性消息的回调函数
 * @param 
 */
void task_server::onAttriRecvd(const custom_msgs::RoadAttri::ConstPtr &msg){
    mtx_road.lock();
    ros_road_attri = *msg;
    mtx_road.unlock();
    
}

/**
 * @brief 接收到道路线消息的回调函数
 * @param 
 */
void task_server::onRoadRecvd(const custom_msgs::LaneLineArray::ConstPtr &msg){
    mtx_road.lock();
    ros_road_line = *msg;
    mtx_road.unlock();
    park_s = getFrenet2(park_x,park_y,ros_road_line.lines[0].x,ros_road_line.lines[0].y,ros_road_line.lines[0].s[0])[0]; 
}

void task_server::onSegmentMsgRecvd(const custom_msgs::CarDirection::ConstPtr &msg) {
    segment_msg = *msg;
}


void task_server::FlagCallBack(const custom_msgs::Astar::ConstPtr &msg) {
    flag_info = *msg;
}
//*******************************************************加锁访问**************************************************************
/**
 * @brief 为避免异常加锁访问定位
 * @param 
 */
inline custom_msgs::NaviData task_server::get_Navi_WithMutex(){
    custom_msgs::NaviData res;
    mtx_navi.lock();
    res = ros_navi_data;
    mtx_navi.unlock();
    return res;
}

/**
 * @brief 为避免异常加锁访问位置
 * @param 
 */
inline custom_msgs::CurPose task_server::get_Curpose_WithMutex(){
    custom_msgs::CurPose res;
    mtx_pose.lock();
    res = ros_cur_pose;
    mtx_pose.unlock();
    return res;
}

/**
 * @brief 为避免异常加锁访问目标
 * @param 
 */
inline custom_msgs::ObjectArray task_server::get_Object_WithMutex(){
    custom_msgs::ObjectArray res;
    mtx_object.lock();
    res = ros_object_arry;
    mtx_object.unlock();
    return res;
}

/**
 * @brief 为避免异常加锁访问道路属性
 * @param 
 */
inline custom_msgs::RoadAttri task_server::get_RoadAttri_WithMutex(){
    custom_msgs::RoadAttri res;
    mtx_road.lock();
    res = ros_road_attri;
    mtx_road.unlock();
    return res;
}

/**
 * @brief 为避免异常加锁访问道路xian
 * @param 
 */
inline int task_server::get_LaneID_WithMutex(){
    int res;
    if(ros_road_line.lines.size()==0)
        return -1;
    mtx_road.lock();
    res = ros_road_line.lines[0].current_lane_num;
    mtx_road.unlock();
    return res;
}


inline custom_msgs::CarDirection task_server::get_CarDirection_WithMutex() {
	custom_msgs::CarDirection res;
    mtx_car_direction.lock();
    res = segment_msg;
    mtx_car_direction.unlock();
    return res;
}

inline custom_msgs::Astar task_server::get_Astar_WithMutex() {
    custom_msgs::Astar res;
    mtx_astar.lock();
    res = flag_info;
    mtx_astar.unlock();
    return res;
}
//*******************************************************车辆控制**************************************************************
/**
 * @brief 设置速度
 * @param 
 */
inline bool task_server::SetSpeed(bool enable,float speed,std::string source){
    custom_msgs::Control srv;
    srv.request.type   = custom_msgs::Control::Request::Is_switch_speed;
    srv.request.enable = enable;
    srv.request.info   = speed;
    srv.request.source = source;
    if (control_client.call(srv)){
        return 1;
    }
    else{
        ROS_ERROR("task_server : Failed to call service contrl");
        return 0;
    }
}

/**
 * @brief 停车控制
 * @param  
 */
inline bool task_server::SetStop(bool enable,float distance,std::string source){
    custom_msgs::Control srv;
    srv.request.type   = custom_msgs::Control::Request::Is_stop;
    srv.request.enable = enable;
    srv.request.info   = distance;
    srv.request.source = source;
    if (control_client.call(srv)){
        // cout<<"control_client111111111111111111111111"<<endl;
        return 1;
    }
    else{
        ROS_ERROR("task_server : Failed to call service contrl");
        return 0;
    }
}

/**
 * @brief AEB使能
 * @param 
 */
inline bool task_server::SetAEB(bool enable,float distance,std::string source){
    custom_msgs::Control srv;
    srv.request.type   = custom_msgs::Control::Request::Is_aeb;
    srv.request.enable = enable;
    srv.request.info   = distance;
    srv.request.source = source;
    if (control_client.call(srv)){
        return 1;
    }
    else{
        ROS_ERROR("task_server : Failed to call service contrl");
        return 0;
    }
}

/**
 * @brief 倒车使能
 * @param 
 */
inline bool task_server::SetBackCar(bool enable,std::string source){
    custom_msgs::Control srv;
    srv.request.type   = custom_msgs::Control::Request::Is_back;
    srv.request.enable = enable;
    srv.request.source = source;
    if (control_client.call(srv)){
        return 1;
    }
    else{
        ROS_ERROR("task_server : Failed to call service contrl");
        return 0;
    }
}

/**
 * @brief 设置路径
 * @param d：平移距离 s:缓急程度，越小越急
 */
inline bool task_server::SetRoute(bool enabel,float d,float s){
    custom_msgs::Route srv;
    srv.request.enable    = enabel;
    srv.request.target_s  = s;
    srv.request.target_d  = d;
    if (route_client.call(srv)){
        return 1;
    }
    else{
        ROS_ERROR("task_server : Failed to call service route");
        return 0;
    }

}

//*******************************************************任务表**************************************************************
/**
 * @brief 根据线程id为任务表添加任务
 * @param 
 */
void task_server::Add_task(std::string str){
    std::stringstream sin;
    sin << std::this_thread::get_id();
    std::string stid = sin.str();
    unsigned long long tid = std::stoull(stid);
    task_table[tid] = str;
}

/**
 * @brief 删除该线程对应的任务表中任务
 * @param 
 */
void task_server::Delete_task(){
    std::stringstream sin;
    sin << std::this_thread::get_id();
    std::string stid = sin.str();
    unsigned long long tid = std::stoull(stid);
    task_table.erase(tid);
}

/**
 * @brief 输出任务表格
 * @param 
 */
void task_server::Show_taskTable(){
    std::stringstream sin;
    sin << std::endl;
    sin << "*****************TASK_TABLE***********************"<<std::endl;
    sin << "*     ThreadID       |       task_describe       *"<<std::endl;
    sin << "*------------------------------------------------*"<<std::endl;
    if(task_table.size()){
        for(auto it = task_table.begin();it != task_table.end();it++){
            sin <<"*  "<<it->first<<"   |  "<<it->second<<std::endl;
        }
    }else{
        sin << "*                    NULL                        *"<<std::endl;
    }
    
    sin << "**************************************************"<<std::endl;
    std::string task_string = sin.str();
    std::cout << task_string << std::endl;
}

//***************************************************任务执行需要的功能函数******************************************************
/**
 * @brief 获得红绿灯信息
 * @param 
 * @return 返回大小为2的vector，分别为颜色与时间 
 */
inline std::vector<int> task_server::Get_traffic_light(int rsu_id,int phase_id){
    custom_msgs::TrafficLight srv;
    std::vector<int> res;
    // srv.request.rsu_id   = rsu_id;
    // srv.request.phase_id = phase_id;
    //srv.request.direction = -90;
    if(!traffic_light_client.call(srv)){
        ROS_ERROR("task_server : obu_node dose not response");
    }
    res.push_back(srv.response.color);
    res.push_back(srv.response.time);
    ROS_INFO_THROTTLE(1,"color: %d, time: %d",srv.response.color,srv.response.time);
    return res;
}

/**
 * @brief 地图切换
 * @param 
 */
inline bool task_server::SetMap(int id){
    custom_msgs::Map_Switch srv;
    srv.request.id = id;
    if (mapsw_client.call(srv)){
        return 1;
    }
    else{
        ROS_ERROR("task_server : Failed to call service map");
        return 0;
    }

}

/**
 * @brief 获取闸门状态
 * @param 
 */
inline bool task_server::Get_gate_stat(int id){
    custom_msgs::GateStat srv;
    srv.request.id = id;
    if (gate_client.call(srv)){
        return srv.response.stat;
    }
    else{
        ROS_ERROR("task_server : Failed to call service gate");
        return 0;
    }

}

/**
 * @brief 判断所选区间是否有目标
 * @param position:0->整个车道 1->左车道 2->右车道
 *        s_cur:车辆位置
 *        front：前方距离
 *        back：后方距离
 */
bool task_server::Is_have_target(int position,float s_cur,float front,float back,custom_msgs::ObjectArray &objectArry){
    
    float road_w = get_RoadAttri_WithMutex().road_width;
    for(const auto &obj : objectArry.objs){
        //整个车道
        // s_cur的前后范围内有目标
        if(position == 0){
            if(obj.s_pos>(s_cur-back) && obj.s_pos<(s_cur+front)) return true;
        }
        //左车道
        // 前后范围内有目标
        if(position == 1){
            if(obj.s_pos>(s_cur-back) && obj.s_pos<(s_cur+front)){
                if(obj.d_pos < -road_w/4) return true;
            }
        }
        //右车道
        // 前后范围内有目标
        if(position == 2){
            if(obj.s_pos>(s_cur-back) && obj.s_pos<(s_cur+front)){
                if(obj.d_pos > -road_w/4) return true;
            }
        }
        // //右车道前方无目标
        // //变道后变回去
        // if(position == 3){
        //     if(obj.s_pos>(s_cur+back) && obj.s_pos<(s_cur+front)){
        //         if(obj.d_pos > -road_w/4) return true;
        //     }
        // } 
    }
    return false;
}

/**
 * @brief 码头检查距离获取
 * @param 
 */
inline float task_server::get_Wharf_dis(bool enable){
    custom_msgs::Wharf srv;
    srv.request.enable = enable;
    if (wharf_client.call(srv)){
        return srv.response.distance;
    }
    else{
        ROS_ERROR("task_server : Failed to call service wharf");
        return 999;
    }
}

/**
 * @brief 码头检查距离获取
 * @param 
 */
inline float task_server::SendMsg2APP(std::string buff){
    custom_msgs::App srv;
    srv.request.isSend = 1;
    srv.request.sendbuf = buff;
    if (link2app_client.call(srv)){
        return 1;
    }
    else{
        ROS_ERROR("task_exe : Failed to call service app");
        return 0;
    }
}

/**
 * @brief 判断区间是否有目标
 * @param 
 */
float task_server::Is_have_target_Instopregion(float cur_s,float s_stop,float front,float back,custom_msgs::ObjectArray &objectArry){
    
    for(const auto &obj : objectArry.objs){
       if(obj.s_pos > s_stop-back && obj.s_pos < s_stop+front && obj.lwh.z>0.8 && obj.s_pos>cur_s)
       return obj.s_pos;
    }
    return -1;
}

/**
 * @brief reboot slam
 * @param :1、node_main.cc  2、occupancy_grid_node_main.cc
 */
 bool task_server::reboot_slam(){

    slam_start_mtx.lock();
    custom_msgs::SLAMReboot srv;
    reboot_slam_client.call(srv);
    reboot_grid_client.call(srv);
    slam_start_mtx.unlock();

    usleep(1000*500);
 //   reboot_location_client.call(srv);
    return true;
 }



//*******************************************************任务处理函数**************************************************************
/**
 * @brief 灯光任务函数
 * @param info：0大灯 1左灯 2右灯
 */
void task_server::Light_mission(float s_start,float s_end,float info){

    Add_task("light_mission");

    custom_msgs::CurPose cur_pose;
    custom_msgs::LowHeadLightCmd srv_lh;
    custom_msgs::TurnLightCmd srv_tl;
    ros::Rate loop_rate(10);

    do{
        cur_pose = get_Curpose_WithMutex();
        ROS_WARN_STREAM(cur_pose.s);
        switch ((int)info)
        {
        case 0:
            srv_lh.request.LowLightEnable = custom_msgs::LowHeadLightCmd::Request::LowTurnOn;
            low_head_light_client.call(srv_lh);
            break;
        case 1:
            srv_tl.request.TurnLightEnable = custom_msgs::TurnLightCmd::Request::TurnLeft;
            turn_light_client.call(srv_tl);
            break;
        case 2:
            srv_tl.request.TurnLightEnable = custom_msgs::TurnLightCmd::Request::TurnRight;
            turn_light_client.call(srv_tl);
            break;
        
        default:
            break;
        }
        loop_rate.sleep();
    }while(cur_pose.s < s_end && cur_pose.s > s_start);

    srv_lh.request.LowLightEnable = custom_msgs::LowHeadLightCmd::Request::LowTurnOff;
    low_head_light_client.call(srv_lh);

    srv_tl.request.TurnLightEnable = custom_msgs::TurnLightCmd::Request::TurnOff;
    turn_light_client.call(srv_tl);

    Delete_task();
}

/**
 * @brief 通过红绿灯任务
 * @param info：停止线
 */
void task_server::Pass_traffic_light_mission(float s_start,float s_end,float info){
    Add_task("Pass_traffic_light_mission");

    custom_msgs::CurPose cur_pose;
    std::vector<int> TrafficLigtht;   //TrafficLigtht[0]颜色；TrafficLigtht[1]剩余时间
    float dist;

    ros::Rate loop_rate(10);

    do{
        loop_rate.sleep();
        cur_pose = get_Curpose_WithMutex();
        //ROS_WARN_STREAM(cur_pose.s);

        TrafficLigtht = Get_traffic_light(1,1);
        //ROS_INFO_STREAM_THROTTLE(1,"color: "<< TrafficLigtht[0]<<" time: "<< TrafficLigtht[1]);
        dist = info - cur_pose.s;
        if(dist > 30){
            continue;
        }
        if(dist > 12){
            SetSpeed(true,8,"Pass_traffic_light_mission");
            continue;
        }
        if(dist > 6){
            if(TrafficLigtht[0] == custom_msgs::TrafficLight::Response::GREEN ){//绿灯且时间充足，直接通过
                SetStop(false,9,"Pass_traffic_light_mission");
            }else{//否则减速
                SetSpeed(true,5,"Pass_traffic_light_mission");
            }
        }
        if(dist < 1.5){
            if(TrafficLigtht[0] == custom_msgs::TrafficLight::Response::GREEN ){//绿灯且时间充足，直接通过
                SetStop(false,9,"Pass_traffic_light_mission");
            }else{//否则刹停
                SetStop(true,9,"Pass_traffic_light_mission");
            }
        }        
    }while(cur_pose.s < s_end && cur_pose.s > s_start);

    SetSpeed(false,10,"Pass_traffic_light_mission");
    SetStop(false,9,"Pass_traffic_light_mission");

    Delete_task();
}

/**
 * @brief 避让障碍物任务
 * @param info：暂未使用
 */
void task_server::Avoid_obstacle_mission(float s_start,float s_end,float info) { 
    Add_task("Avoid_obstacle_mission");
    
    custom_msgs::CurPose     cur_pose;
    custom_msgs::ObjectArray objsArry;
    custom_msgs::NaviData    navi_data;
    float                    road_w;
    int ischanging = 0;
    ros::Rate loop_rate(10);
    
    do{
        loop_rate.sleep();
        cur_pose  = get_Curpose_WithMutex();
        navi_data = get_Navi_WithMutex();
        objsArry  = get_Object_WithMutex();
        road_w    = get_RoadAttri_WithMutex().road_width;

        if(fabs(cur_pose.d) >= 1 && navi_data.speed2d>1) {   
            std::cout << "________in1" << std::endl;                         
            if(!Is_have_target(0, cur_pose.s, 3, 2,objsArry) && ischanging!=0) {      // 全车道没有目标，变道回去
                  std::cout << "________in2" << std::endl; 
                  SetRoute(false,-road_w/2, 3);
                  ischanging = 0;
                ROS_WARN_STREAM_THROTTLE(2, "once change road finished!______________________");
            }
        }

        if(fabs(cur_pose.d) <= 1 && navi_data.speed2d>1) {       // 车辆跟随正常行驶, 左车道 前方出现障碍物
            if (Is_have_target(1, cur_pose.s, 5, 2, objsArry) && ischanging!=2) { 
                SetRoute(true, road_w/3, 3);    // 右变道
                ischanging = 1;
                ROS_WARN_STREAM_THROTTLE(2, "Right change road finished!______________________");
            }
        } 

        if(fabs(cur_pose.d) <= 1 && navi_data.speed2d>1) {       // 车辆跟随正常行驶, 右车道 前方出现障碍物
            if (Is_have_target(2, cur_pose.s, 5, 2, objsArry) && ischanging!=1) { 
                SetRoute(true, -road_w/3, 3);    // 左变道
                ischanging = 2;
                ROS_WARN_STREAM_THROTTLE(2, "Left change road finished!______________________");
            }
        }

        // if ((ischanging==1 || ischanging==2) && navi_data.speed2d>1) {
        //     if (Is_have_target(ischanging, cur_pose.s, 8, 2, objsArry)) {

        //         SetAEB(true, 1, "Avoid_obstacle_mission");
        //     }else {
        //         SetAEB(false, 1, "Avoid_obstacle_mission");
        //     }
        // }

    }while(cur_pose.s < s_end && cur_pose.s > s_start);
    SetRoute(false, road_w/3, 3);
    Delete_task();
}

// /**
//  * @brief 倒车任务
//  * @param info：
//  */
// void task_server::Back_car_mission(float s_start,float s_end,float info ,float info_2){
//     Add_task("Back_car_mission");

//     custom_msgs::CurPose  cur_pose;
//     custom_msgs::NaviData navi_data;
//     ros::Rate loop_rate(10);
//     float dis_front,dis_back,dis_wharf;
//     bool back_pocess = false;
//      bool is_direct = false;
//     int map_id = -1;
//     while(map_id == -1){
//         map_id = get_LaneID_WithMutex();
//         ros::Duration(0.5).sleep();
//     }

//     do{
//         loop_rate.sleep();
//         cur_pose = get_Curpose_WithMutex();
//         navi_data = get_Navi_WithMutex();
//         //dis_wharf = get_Wharf_dis(true);
//         dis_front = info - cur_pose.s;
//         dis_back  = info_2 - cur_pose.s;

//         ROS_INFO_STREAM_THROTTLE(2,"map_id    :"<<map_id);
//         ROS_INFO_STREAM_THROTTLE(2,"dis_front :"<<dis_front);
//         ROS_INFO_STREAM_THROTTLE(2,"dis_back  :"<<dis_back);
//         ROS_INFO_STREAM_THROTTLE(2,"dis_wharf :"<<dis_wharf);
        
//         if(back_pocess == false && dis_front < 2){
// 	    ROS_INFO_STREAM_THROTTLE(1,"111111");
//             SetStop(true,9,"Back_car_mission_front");
//             SetBackCar(true,"Back_car_mission");
//             ros::Duration(2).sleep();
//             SetMap(map_id+2);
//             ros::Duration(2).sleep();
//             SetSpeed(true,-2,"Back_car_mission");
//             SetStop(false,9,"Back_car_mission_front");
//             back_pocess = true;
//         }
        
//         if(dis_back < 0.4){
// 	    ROS_INFO_STREAM_THROTTLE(1,"222222");
//             SetStop(true,9,"Back_car_mission_back");
//             SetBackCar(false,"Back_car_mission");
//             ros::Duration(2).sleep();
//             SetMap(map_id+4);
//             ros::Duration(2).sleep();
//             SetStop(false,9,"Back_car_mission_back");
//             // SetStop(true,7,"Back_car_mission_back");
//             // ros::Duration(1).sleep();
//             break;
//         }
//         if(map_id+2 == 7 && dis_back <8 ){
//             ROS_INFO_STREAM_THROTTLE(1,"3333333");
//             dis_wharf = get_Wharf_dis(true);
//             if(dis_wharf < 0.7){
//                 SetStop(true,7,"Back_car_mission_back");
//                 ros::Duration(1).sleep();
//                 //break;
//             } 
//         }
//         if(map_id+2 == 7 && dis_back > 8 && dis_back<25 && !is_direct){
//             ROS_INFO_STREAM_THROTTLE(2,"revise runing");
//             SetRoute(true,0.2, 1);
//         }
//         if(map_id+2 == 7 && dis_back < 8){
//             SetRoute(true,0, 0);
//             ROS_INFO_STREAM_THROTTLE(2,"direct runing");
//             is_direct = true;
//         }
       
//     }while(cur_pose.s < s_end && cur_pose.s > s_start);

//     SetBackCar(false,"Back_car_mission");
//     SetRoute(false,0, 0);
//     SetSpeed(false,2,"Back_car_mission");
//     get_Wharf_dis(false);
    
//     // SetStop(false,7,"Back_car_mission_back");

//     Delete_task(); 
// }

/**
 * @brief hybrid_astar倒车任务
 * @param info：
 */
void task_server::Back_car_mission(float s_start,float s_end,float info ,float info_2){
    //主要使用 s_start、s_end
    Add_task("Back_car_mission");
    custom_msgs::CarDirection car_pose;
    custom_msgs::CurPose  cur_pose;
    custom_msgs::NaviData navi_data;
    
    float dis_front,dis_back;
    bool forward_pocess = false;
    bool back_pocess = false;
    custom_msgs::VehicleStat  stop_stat;
    stop_stat.isStopCar = false; 


    ros::Rate loop_rate(10);
    do{
        loop_rate.sleep();
        cur_pose = get_Curpose_WithMutex();
        navi_data = get_Navi_WithMutex();
        car_pose = get_CarDirection_WithMutex();

        dis_front = cur_pose.s - s_start;
        dis_back  = s_end - cur_pose.s; //当前路径最后点的s-车当前的s

        ROS_INFO_STREAM_THROTTLE(1,"----s_end  : "<< s_end);
        ROS_INFO_STREAM_THROTTLE(1,"------cur_pose.s : "<< cur_pose.s);

            //前进，到停车点，停下，发布车已经停下，发出标志位跳出循环，删除任务
            if (car_pose.direction == 1) {  // =1为前进
                ROS_INFO_STREAM_THROTTLE(1,"----dis_back  : "<< dis_back);
                ROS_INFO_STREAM_THROTTLE(1,"forward");

                if (!forward_pocess) { //设置速度
                    SetSpeed(true,2,"forward_mission");
                    ROS_INFO_STREAM_THROTTLE(1,"SetSpeed");
                    forward_pocess = true;
                }
                
                if (dis_back < 0.5) { //停车
                    SetStop(true,9,"forward_mission");
                    SetSpeed(true,0,"forward_mission");
                    
                    ROS_INFO_STREAM_THROTTLE(1,"SetStop");
                    stop_stat.isStopCar = true;
                        
                }
                
                if (dis_back > 1.5 && dis_back < 3) {   //减速
                    ROS_INFO_STREAM_THROTTLE(1,"slow Speed");
                    SetSpeed(true,1,"forward_mission"); 
                    ROS_INFO_STREAM_THROTTLE(1,"************slow speed*******************");
                }
                
                // stop_car_status_pub.publish(stop_stat);
                // ROS_INFO("Published stop car status:isStopCar = %d",stop_stat.isStopCar);
                if (stop_stat.isStopCar) {
                    SetStop(false,9,"forward_mission");
                    SetSpeed(false,1,"forward_mission");
                    stop_car_status_pub.publish(stop_stat);
                    ROS_INFO_STREAM_THROTTLE(1,"******forward_mission Jump out*********");
                    stop_stat.isStopCar = false;
                    break;    
                }

            }
            //后退，到停车点，停下，发布车已经停下，发出标志位跳出循环，删除任务
            if (car_pose.direction == -1 ) {
                ROS_INFO_STREAM_THROTTLE(1,"Back_car");
                if (back_pocess == false) { //换挡设置速度
                    SetSpeed(true,0,"Back_car_mission");
                    SetBackCar(true,"Back_car_mission");
                    ros::Duration(1).sleep();
                    ROS_INFO_STREAM_THROTTLE(1,"start Back_car");
                    SetSpeed(true,-2,"Back_car_mission");
                    back_pocess = true;
                }
                cout<<"dis_back = "<<dis_back<<endl;
                // ROS_INFO_STREAM_THROTTLE(1,"*********jump******************" << dis_back);
                if(back_pocess && dis_back < 1){
                    ROS_INFO_STREAM_THROTTLE(1,"111111");
                    SetStop(true,9,"Back_car_mission");
                    SetSpeed(true,0,"Back_car_mission");
                    
                    stop_stat.isStopCar = true;    
                } 
                stop_car_status_pub.publish(stop_stat);
                if (stop_stat.isStopCar) {
                    // stop_car_status_pub.publish(stop_stat);
                    // ros::Duration(1).sleep();
                    SetStop(false,9,"Back_car_mission");
                    SetBackCar(false,"Back_car_mission");
                    SetSpeed(false,2,"Back_car_mission");
                    stop_stat.isStopCar = false;
                    ROS_INFO_STREAM_THROTTLE(1,"*********jump******************");
                    break;
                }
            }
            // ROS_INFO_STREAM_THROTTLE(1,"isForwardSlow "<<isForwardSlow);
        
    }while((cur_pose.s < s_end ));
    ROS_INFO_STREAM_THROTTLE(1,"******mission finish*********");
    ROS_INFO_STREAM_THROTTLE(1,"dis_back-----  : "<< dis_back);
    Delete_task(); 

}

/**
 * @brief HybridAstar stop car

 * @param info：
 */

void task_server::stop_run_car(float s_start,float s_end,float info){

    Add_task("stop_car_mission");
    custom_msgs::CarDirection car_pose;
    custom_msgs::CurPose  cur_pose;
    custom_msgs::NaviData navi_data;
    custom_msgs::Astar is_new_planning;
    float dist_front,dist_back; 
    bool stopFlag = false;
    bool stop_pocess = false;
    ros::Rate loop_rate(10);
    
    
    
    do{
        
        loop_rate.sleep();
        cur_pose = get_Curpose_WithMutex();
        navi_data = get_Navi_WithMutex();
        car_pose = get_CarDirection_WithMutex();
        is_new_planning = get_Astar_WithMutex();
        ROS_INFO_STREAM_THROTTLE(1,"----cur_pose.s : "<< cur_pose.s);
        ROS_INFO_STREAM_THROTTLE(1,"----s_end  : "<< s_end);
        dist_front = cur_pose.s - s_start;
        dist_back = s_end - cur_pose.s;

        if (car_pose.lastSegment == 1) {//规划路径只有一段或者是最后一段规划的路径，停车速度设置为0
            ROS_INFO_STREAM_THROTTLE(1,"into_stop_car");
            
            if (dist_back > 0) {
                if (!stop_pocess) {
                    if (car_pose.direction == 1) {  //前进 设置速度
                        SetSpeed(true,2,"stop_car_mission");
                    } else {    //后退 设置速度
                        SetBackCar(true,"stop_car_mission");
                        ros::Duration(0.5).sleep();
                        SetSpeed(true,-2,"stop_car_mission");
                    }
                    stop_pocess = true;
                }

                if(stop_pocess && dist_back < 0.2){ //停车
                    SetStop(true,5,"stop_car_mission");
                    SetSpeed(true,0,"stop_car_mission");
                    if (car_pose.direction == -1) { //倒车
                        SetBackCar(false,"stop_car_mission");
                    }
                    ROS_INFO_STREAM_THROTTLE(1,"-----dist_back --- : "<< dist_back);

                }
                    
                if(dist_back > 1 && dist_back < 3){   //减速
                // ROS_INFO_STREAM_THROTTLE(1,"111111");
                    // SetStop(true,5,"stop_car_mission");
                    if (car_pose.direction == 1) {  //前进 减速
                        SetSpeed(true,1,"stop_car_mission");
                        ROS_INFO_STREAM_THROTTLE(1,"-----dist_back --- : "<< dist_back);
                    } else {    //后退 减速
                        SetSpeed(true,-2,"stop_car_mission");
                    }
                    
                }
                
                if (is_new_planning.new_plan_executing) {   //当重新开始进行新的一次规划时，解除上一次停车状态
                    SetStop(false,5,"stop_car_mission");
                    is_new_planning.new_plan_executing = false;
                    stopFlag = true;
                }
            }
        }
            
   
    }//while(!stopFlag && (cur_pose.s < s_end && cur_pose.s > s_start));
    while(!stopFlag && (cur_pose.s < s_end));


    // SetStop(false,9,"stop_car_mission");
    SetSpeed(false,2,"stop_car_mission");
    ROS_INFO_STREAM_THROTTLE(1,"cur_pose.s --- : "<< cur_pose.s);
    Delete_task();
}

/**
 * @brief 闸门任务
 * @param info：
 */
void task_server::Pass_gate_mission(float s_start,float s_end,float info){
    Add_task("li rang xing ren");

    custom_msgs::CurPose cur_pose;
    custom_msgs::ObjectArray objsArry;
    ros::Rate loop_rate(10);
    float dist = 0;
    bool  ishave = false;

    do{
        loop_rate.sleep();
        cur_pose = get_Curpose_WithMutex();
        objsArry = get_Object_WithMutex();
        ishave = Is_have_target(0,cur_pose.s, 7, 0,objsArry);
        dist = info - cur_pose.s;

        if(dist < 2 && ishave){
            SetStop(true,10,"li rang xing ren");
        }else{
            SetStop(false,10,"li rang xing ren");
        }         
    }while(cur_pose.s < s_end && cur_pose.s > s_start);

    SetStop(false,7,"li rang xing ren");

    Delete_task();
}

/**
 * @brief change map
 * @param info：
 */
void task_server::Change_map(float s_start,float s_end,float info){
    Add_task("change_map_mission");

    SetStop(true,7,"change_map_mission");
    SetMap((int)info);
    ros::Duration(4).sleep();
    SetStop(false,7,"ALL");

    if((int)info == 9 && get_Navi_WithMutex().pose_type != 50){
        SetRoute(true,0, 0);
        while(get_Navi_WithMutex().speed2d <2){
            ros::Duration(1).sleep();
            ROS_INFO_STREAM_THROTTLE(2,"wait runing: " << get_Navi_WithMutex().speed2d);  
        }
        int time_diret = 0;
        while((time_diret++)<12){
            ROS_INFO_STREAM_THROTTLE(2,"direct runing"); 
            ros::Duration(1).sleep();
            ROS_INFO_STREAM_THROTTLE(1,"navi_stat: " << get_Navi_WithMutex().pose_type);
        }
        SetRoute(false,0, 0);
        while(get_Navi_WithMutex().pose_type != 50){
            ros::Duration(1).sleep();
            ROS_INFO_STREAM_THROTTLE(2,"wait navi_stat: " << get_Navi_WithMutex().pose_type);
            SetStop(true,7,"wait_navi_stat");
        }
        SetStop(false,7,"wait_navi_stat");
    }
    
    Delete_task();
}

/**
 * @brief  现在为slam地图切换 调段 
 * @param info： 
 */
void task_server::slam_changeMap(float s_start,float s_end,float info) {
    Add_task("slam_changeMap_mission");


    custom_msgs::CurPose cur_pose;
    ros::Rate loop_rate(10);
    cur_pose = get_Curpose_WithMutex();    
    float dist=info-cur_pose.s;
    do{
        loop_rate.sleep();
        cur_pose = get_Curpose_WithMutex();
        dist=info-cur_pose.s;
        ROS_INFO_STREAM_THROTTLE(2,"dist: " << dist); 
        if(dist<0.5){ 
            SetStop(true,10,"slam_changeMap_mission");
            SetMap(3);      
            sleep(3);
            SetStop(false,10,"slam_changeMap_mission");
            break; 
        }      
         
    } while(cur_pose.s < s_end && cur_pose.s > s_start);    //车库室内切换点： 
    Delete_task();
}


// /**
//  * @brief 

//  * @param info：
//  */
// void task_server::stop_run_car(float s_start,float s_end,float info){
//     Add_task("follow_car_mission");

//     custom_msgs::CurPose     cur_pose;
//     custom_msgs::ObjectArray objsArry;
//     ros::Rate loop_rate(10);

//     float speed = 10;
   
//     do{
//         loop_rate.sleep();
//         cur_pose = get_Curpose_WithMutex();
//         objsArry  = get_Object_WithMutex();
//         if(Is_have_target(2,cur_pose.s, 15, 0,objsArry)){
//             for(const auto &obj : objsArry.objs){
//                 if(obj.s_pos > cur_pose.s && obj.v > 1){
//                     SetSpeed(true,obj.v,"follow_car");
//                 }
//                 break;
//             }  
//         }else{
//             SetSpeed(false,10,"follow_car");
//         }
        

   
//     }while(cur_pose.s>s_start&&cur_pose.s<s_end); 
//     SetSpeed(false,10,"follow_car");
   
//     Delete_task();
// }




/**
 * @brief 


 * @param info：
 */
void task_server::find_obj(float s_start,float s_end,float info){
    Add_task("find_obj_mission");

    custom_msgs::CurPose     cur_pose;
    custom_msgs::ObjectArray objsArry;
    float dist,stop_s,people_s;
    ros::Rate loop_rate(10);
    bool isfind=false;
    bool istop = false;

    SetSpeed(true,5,"fin_obj");

    park_s = info;
    stop_s = park_s+4;
    do{
        loop_rate.sleep();
        cur_pose  = get_Curpose_WithMutex();
        objsArry  = get_Object_WithMutex();


        dist = stop_s - cur_pose.s;
        //ROS_WARN_STREAM_THROTTLE(2,"stop_s: " << stop_s);
        people_s = Is_have_target_Instopregion(cur_pose.s,park_s,20,20,objsArry);
        //ROS_ERROR_STREAM("people_s is find !!!!  : " << people_s);
        if(people_s > 0 && people_s>cur_pose.s && isfind==false){
            stop_s = people_s;
            isfind = true;
            //ROS_ERROR_STREAM("2222 !!!!  : " << people_s);
        }

        if(dist < 1 && istop==false){
            SetStop(true,10,"find_obj_mission");
            istop = true;
        }
        if(dist < 1 && istop==true){
            ros::Duration(4).sleep();
            SetStop(false,10,"find_obj_mission");
        }



        // for(const auto &obj : objsArry.objs){
        //     if(obj.s_pos<cur_pose.s){
        //         continue;
        //     }
        //     else{
        //         info=obj.s_pos;
        //         dist=info-cur_pose.s;
        //         break;
        //     }
        // }
        // cout<<"-------info: "<<info<<endl;
        // cout<<"-------dist: "<<dist<<endl;
        // if(dist<=0.5){
        //     SetStop(true,10,"find_obj_mission");
 
        //     ros::Duration(5).sleep();


        //     flag=false;
        // }
    }while(cur_pose.s>s_start&&cur_pose.s<s_end);
    SetStop(false,10,"find_obj_mission");
    SetSpeed(false,5,"fin_obj");
    Delete_task();
}

/**
 * @brief start slam
 * @param 
 */
void task_server::start_slam_mission(float s_start,float s_end,float info){
    Add_task("start_slam_mission"); 
    
    custom_msgs::CurPose     cur_pose;
    float dist;
    ros::Rate loop_rate(10);
    
    if (!ros::param::has("/Is_slam_start"))
    {
        ROS_WARN("ros param /Is_slam_start not found, the SLAM will not work");
    }

    do{
        loop_rate.sleep();
        cur_pose = get_Curpose_WithMutex(); 
        dist = info-cur_pose.s;
        ros::param::get("/Is_slam_start", Is_slam_start);

        if (exec_time==0) {
            if (dist<5 && dist>0.5) {     // 到达slam启动点5m范围，减速
                SetSpeed(true, 3, "start_slam_mission");                
                ROS_INFO("______________low speed------will start slam!");   
            }

            if (dist < 0.5 && Is_slam_start==0 ){   

                // slam_start_mtx.lock();
                SetStop(true, 3, "start_slam_mission"); 
                SetSpeed(true, 3, "start_slam_mission");
                ros::param::set("/Is_slam_start", 1); 
                
                // int ret = system("roslaunch ${SEED_HOME}/src/real_time_slam_location.launch&"); //必须在.bashrc里面source
                // ROS_INFO("Sys Start.launch run flag: %d", ret); // 0: success
                // ROS_INFO("______________SLAM------Start-------Success!");        
                // slam_start_mtx.unlock(); 
            
            }else if(dist < 0.5 && Is_slam_start==2 ) {
                ros::param::set("/Is_slam_start", 3); 
                slam_start_mtx.lock();
                int ret = system("roslaunch ${SEED_HOME}/src/real_time_slam_location.launch&"); //必须在.bashrc里面source
                slam_start_mtx.unlock(); 
                if(ret!=0){
                    ROS_INFO("______________SLAM------Start-------Failed!"); 
                    SetStop(true, 3, "start_slam_mission"); 
                    exec_time=0;

                } else{

                    // ROS_INFO("Sys Start.launch run flag: %d", ret); // 0: success
                    ROS_INFO("______________SLAM------Start-------Success!");   
                    sleep(3);
                    exec_time=1;
                    SetStop(false, 10, "start_slam_mission"); 
                    break;

                }

            }
        }
    }while(cur_pose.s>s_start&&cur_pose.s<s_end);
    Delete_task(); 
}
/**
 * @brief close slam
 * @param 
 */
void task_server::close_slam_mission(float s_start,float s_end,float info){
    Add_task("close_slam_mission"); 

    custom_msgs::CurPose     cur_pose;
    custom_msgs::NaviData    pose_type;
    float dist;
    ros::Rate loop_rate(10);

    if (!ros::param::has("/Is_slam_start"))
    {
        ROS_WARN("ros param /Is_slam_start not found, the SLAM will not work");
    }

    do{
        loop_rate.sleep();
        cur_pose = get_Curpose_WithMutex();  
        pose_type = get_Navi_WithMutex();
        dist = info-cur_pose.s;
        ros::param::get("/Is_slam_start", Is_slam_start);  
 
        if( dist < 0.5 && Is_slam_start==3 && exec_time>=300) {  
            
            SetStop(true, 2, "close_slam_mission"); 
            SetSpeed(false, 2, "close_slam_mission");
            ros::param::set("/Is_slam_start", 0); ros::param::get("/Is_slam_start", Is_slam_start);
            //ros::param::set("/slam_origin_get_once", false); ros::param::get("/slam_origin_get_once", slam_origin_get_once);
            // int ret = task_server::reboot_slam(); // 内部已经加锁 
            slam_start_mtx.lock();
            int ret = system("sh ${SEED_HOME}/src/reboot_slam.sh&"); //必须在.bashrc里面source
            slam_start_mtx.unlock();
            // ROS_INFO("Sys Close.sh run flag: %d", ret); 

            if(ret!=0) { 
                ROS_INFO("______________SLAM----Close----Failed!");  
                SetStop(true, 10, "close_slam_mission");
            }else {     

                ROS_INFO("______________SLAM----Close----Success!"); 
                while(pose_type.pose_type!=42 && pose_type.pose_type!=52) {
                // while(pose_type.pose_type!=42) {
                    int print_cnt=0;
                    SetStop(true, 10, "close_slam_mission");
                    if (print_cnt++%10==0) {
                        std::cout << "_____Gnss pose_type: " << pose_type.pose_type << std::endl;
                        std::cout << "_____Gnss pose_type error! Wait sys reset..." << std::endl;
                        print_cnt=0;
                    }
                }
                sleep(3); 
                exec_time=0;
                SetStop(false, 3, "close_slam_mission");
                break; 
            } 
        }

    }while(cur_pose.s>s_start&&cur_pose.s<s_end);
    Delete_task();
}

//*******************************************************主函数**************************************************************
/**
 * @brief 主函数
 * @param 
 */
int main(int argc, char **argv){ 

    ros::init(argc, argv, "task_server");
    ros::Time::init();
    ros::Rate loop_rate(10);
    task_server core;
    int count = 0;
    exec_time=0;

    if(ros::param::has("~park_lat"))
        ros::param::get("~park_lat",core.park_lat);
    else{
        ROS_WARN("ros param park_lat not found");
    }
    if(ros::param::has("~park_lon"))
        ros::param::get("~park_lon",core.park_lon);
    else{
        ROS_WARN("ros param park_lon not found");
    }


    core.park_x = 1;
    CoordPoint map_ori;
    map_ori.x_lon = 106.82990612;   //地图原点
    map_ori.y_lat = 29.71371121;
    VehPose pos_map;
    VehPose pose_gps(core.park_lat,core.park_lon,0);
    trans_pose_gps2coord(map_ori,pose_gps,pos_map);
    core.park_x = pos_map.x_lon;    //停车点？ 
    core.park_y = pos_map.y_lat;

    //cout<< "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ " << core.park_x<< " " << core.park_y<< endl;


    while(ros::ok()) {

        count++;
        if(count >= 100){
            core.Show_taskTable();
            count = 0;
        }
        if(exec_time!=0){
            exec_time++;
            if(exec_time>=300){
                exec_time=300+1;
            }
        }else{
            exec_time = 0;
        }

        ros::spinOnce();
        loop_rate.sleep();  
    }
    
    return 0;
}



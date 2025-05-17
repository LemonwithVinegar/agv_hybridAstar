#ifndef TASK_SERVER_H
#define TASK_SERVER_H

#include <ros/ros.h>
#include <ros/time.h>
#include <custom_msgs/NaviData.h>
#include <custom_msgs/Task.h>
#include "custom_msgs/ObjectArray.h"
#include "custom_msgs/LowHeadLightCmd.h"
#include <custom_msgs/TurnLightCmd.h>
#include "custom_msgs/TrafficLight.h"
#include "custom_msgs/GateStat.h"
#include "custom_msgs/Map_Switch.h"
#include <custom_msgs/CurPose.h>
#include <custom_msgs/Control.h>
#include <custom_msgs/Route.h>
#include <custom_msgs/RoadAttri.h>
#include <custom_msgs/LaneLine.h>
#include <custom_msgs/LaneLineArray.h>
#include <custom_msgs/Wharf.h> 
#include "custom_msgs/App.h"
#include <thread>
#include <mutex>
#include <iostream>
#include <map>
#include <string>
#include <sstream>
#include <iomanip>
#include <std_msgs/Bool.h>
#include <compute/trans_coord.h>
#include <geometry_msgs/Pose2D.h>
#include <compute/frenet.h>
#include <custom_msgs/SLAMReboot.h>
#include <custom_msgs/VehicleStat.h>
#include "custom_msgs/CarDirection.h"
#include "custom_msgs/Astar.h"

class task_server{
    
public:
    task_server(){
        task_exc_service = nh.advertiseService("/task_exc_service", &task_server::onTaskCallRecvd,this);

        stop_car_status_pub = nh.advertise<custom_msgs::VehicleStat>("/stop_stat", 1, true);

        sub_navi         = nh.subscribe("/navi_msg",1,&task_server::onNaviMsgRecvd,this);
        sub_cur_pose     = nh.subscribe("/cur_pose_all", 1 , &task_server::onCurPoseSDRecvd, this);
        sub_target       = nh.subscribe("/ass_object", 1 , &task_server::onTargetRecvd, this);
        attri_sub        = nh.subscribe("/road_attri_msg", 1, &task_server::onAttriRecvd, this);
        sub_road         = nh.subscribe("/road_lane", 1, &task_server::onRoadRecvd, this);
        sub_segment_msg     = nh.subscribe("/car_direction", 1 , &task_server::onSegmentMsgRecvd, this);
        sub_astar_path_info = nh.subscribe("/path_info", 1, &task_server::FlagCallBack,this);

        traffic_light_client  = nh.serviceClient<custom_msgs::TrafficLight>("traffic_light_service");  //红绿灯
        turn_light_client     = nh.serviceClient<custom_msgs::TurnLightCmd>("TurnLightCmd");           //转向灯、双闪
        low_head_light_client = nh.serviceClient<custom_msgs::LowHeadLightCmd>("LowHeadLightCmd");     //近远光灯
        control_client        = nh.serviceClient<custom_msgs::Control>("/control_service");                //横向规划
        route_client          = nh.serviceClient<custom_msgs::Route>("/routing_service");                  //纵向规划
        mapsw_client          = nh.serviceClient<custom_msgs::Map_Switch>("/map_switch_server");        //地图切换
        gate_client           = nh.serviceClient<custom_msgs::GateStat>("/gate_service");    
        wharf_client          = nh.serviceClient<custom_msgs::Wharf>("/wharf_measure_service"); 
        link2app_client       = nh.serviceClient<custom_msgs::App>("/link2app_service");

        //reboot_location_client = nh.serviceClient<custom_msgs::SLAMReboot>("location_reboot");    //slam重启
        reboot_slam_client = nh.serviceClient<custom_msgs::SLAMReboot>("slam_reboot");      //cartographer发布
        reboot_grid_client = nh.serviceClient<custom_msgs::SLAMReboot>("grid_reboot");

    }
   
private:
    ros::NodeHandle nh; 

    //任务执行服务
    ros::ServiceServer task_exc_service;

    //v2x客户端
    ros::ServiceClient traffic_light_client;

    //下位机灯光客户端
    ros::ServiceClient turn_light_client;
    ros::ServiceClient low_head_light_client;

    //
    ros::ServiceClient mapsw_client;
    ros::ServiceClient gate_client;
    ros::ServiceClient wharf_client;
    ros::ServiceClient link2app_client;

    //slam用于定位切换的服务
   // ros::ServiceClient reboot_location_client;
    ros::ServiceClient reboot_slam_client;  //关闭cartographer_node
    ros::ServiceClient reboot_grid_client;  //关闭cartographer_occupancy_grid_node

    //routing提供的控制与规划
    ros::ServiceClient control_client;
    ros::ServiceClient route_client;
    bool SetSpeed(bool enable,float speed,std::string source);
    bool SetStop(bool enable,float distance,std::string source);
    bool SetAEB(bool enable,float distance,std::string source);
    bool SetBackCar(bool enable,std::string source);
    bool SetRoute(bool enabel,float d,float s);

    //消息订阅器
    ros::Subscriber sub_navi;
    ros::Subscriber sub_cur_pose;
    ros::Subscriber sub_target;
    ros::Subscriber attri_sub;
    ros::Subscriber sub_road;
    ros::Subscriber sub_segment_msg;
    ros::Subscriber sub_astar_path_info;

    ros::Publisher stop_car_status_pub;



    //回调函数
    bool onTaskCallRecvd(custom_msgs::Task::Request &req,
                         custom_msgs::Task::Response &res);
    void onNaviMsgRecvd(const custom_msgs::NaviData::ConstPtr &msg);
    void onCurPoseSDRecvd(const custom_msgs::CurPose::ConstPtr &msg);
    void onTargetRecvd(const custom_msgs::ObjectArray::ConstPtr &msg);
    void onAttriRecvd(const custom_msgs::RoadAttri::ConstPtr &msg);
    void onRoadRecvd(const custom_msgs::LaneLineArray::ConstPtr &msg);
    void onSegmentMsgRecvd(const custom_msgs::CarDirection::ConstPtr &msg);
    void FlagCallBack(const custom_msgs::Astar::ConstPtr &msg);

private:
    std::mutex               mtx_navi,mtx_pose,mtx_object,mtx_road,mtx_roadID,mtx_car_direction,mtx_astar;
    std::mutex               slam_start_mtx;        

    custom_msgs::NaviData    ros_navi_data;
    custom_msgs::CurPose     ros_cur_pose;
    custom_msgs::ObjectArray ros_object_arry;
    custom_msgs::RoadAttri   ros_road_attri;
    custom_msgs::LaneLineArray ros_road_line;
    custom_msgs::CarDirection segment_msg;
    custom_msgs::Astar flag_info;
    custom_msgs::NaviData    get_Navi_WithMutex();
    custom_msgs::CurPose     get_Curpose_WithMutex();
    custom_msgs::ObjectArray get_Object_WithMutex();
    custom_msgs::RoadAttri   get_RoadAttri_WithMutex();
    int                      get_LaneID_WithMutex();
    custom_msgs::CarDirection    get_CarDirection_WithMutex();
    custom_msgs::Astar get_Astar_WithMutex();

private:
    //任务处理函数 
    void Light_mission(float s_start,float s_end,float info);
    void Pass_traffic_light_mission(float s_start,float s_end,float info);
    void Avoid_obstacle_mission(float s_start,float s_end,float info);
    void Back_car_mission(float s_start,float s_end,float info,float info_2);
    void Pass_gate_mission(float s_start,float s_end,float info);
    void Change_map(float s_start,float s_end,float info);
    void slam_changeMap(float s_start,float s_end,float info);
    void stop_run_car(float s_start,float s_end,float info);
    void find_obj(float s_start,float s_end,float info);

    void start_slam_mission(float s_start,float s_end,float info);
    void close_slam_mission(float s_start,float s_end,float info);

    //任务执行需要的功能函数  基础函数
    bool Is_have_target(int position,float s_cur,float front,float back,custom_msgs::ObjectArray &objectArry);
    bool SetMap(int id);
    std::vector<int> Get_traffic_light(int rsu_id,int phase_id);
    bool  Get_gate_stat(int id);
    float get_Wharf_dis(bool enable);
    float SendMsg2APP(std::string buff);
    float Is_have_target_Instopregion(float cur_s,float s_stop,float front,float back,custom_msgs::ObjectArray &objectArry);
    bool reboot_slam();
    int Is_slam_start;

public:
    //任务表
    std::map<unsigned long long,std::string> task_table;
    void Add_task(std::string str);
    void Delete_task();
    void Show_taskTable();

    //停车？
    double park_lat;
    double park_lon;
    double park_x;
    double park_y;
    double park_s;

};

#endif

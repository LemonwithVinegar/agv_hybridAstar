#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>

#include "tcp_core.h"
#include "msg.h"
#include <thread>
#include <ros/ros.h>
#include <ros/time.h>
#include "custom_msgs/VehicleStat.h"
#include "custom_msgs/NaviData.h"
#include "custom_msgs/WarnCode.h"
//#include "custom_msgs/AebTime.h"
#include "custom_msgs/Map_Switch.h"
#include <custom_msgs/CurPose.h>
#include "custom_msgs/Control.h"
#include <custom_msgs/Task.h>
#include <custom_msgs/LaneLine.h>
#include <custom_msgs/LaneLineArray.h>
using namespace std;

tcp_core core;
int uart;
char recv_buf[1024];

VichleStatu     vichle_statu;
State_Control   state_control;
Task_Send       task_send;
Task_Res        task_res;
ros::ServiceClient  map_pub_client;
ros::ServiceClient  control_client;
ros::ServiceClient  mapsw_client;
custom_msgs::Task map_chge;
custom_msgs::LaneLineArray ros_road_line;
custom_msgs::CurPose     ros_cur_pose;
int map_id=-1;

void onNaviMsgRecvd(const custom_msgs::NaviData::ConstPtr &msg){
    vichle_statu.direct   = (*msg).heading;
    vichle_statu.lon =  (*msg).longitude;
    vichle_statu.lat  = (*msg).latitude;
}
// void onAebtMsgRecvd(const custom_msgs::AebTime::ConstPtr &msg)
// {
//     aeb_time=(*msg).aeb_time;
// }
void map_switch(int id)
{
    map_chge.request.task_type = 6; 
    map_chge.request.s_start = 0;
	map_chge.request.s_end = 0;
	map_chge.request.info = id;
	map_chge.request.info_2 = 0;
    if(map_pub_client.call(map_chge)) {
        while(map_chge.response.isSuccess == false){
		ROS_ERROR("task_exc_service's response is failed");
		ros::Duration(2).sleep();
		map_pub_client.call(map_chge);
	    }
    }
    else ROS_ERROR("task_exc_service's response is failed");
}
bool SetAEB(bool enable,float distance,std::string source){
    custom_msgs::Control srv;
    srv.request.type   = custom_msgs::Control::Request::Is_aeb;
    srv.request.enable = enable;
    srv.request.info   = distance;
    srv.request.source = source;
    if (control_client.call(srv)){
        return 1;
    }
    else{
        ROS_ERROR("Failed to call service contrl");
        return 0;
    }
}
bool SetStop(bool enable,float distance,std::string source){
    custom_msgs::Control srv;
    srv.request.type   = custom_msgs::Control::Request::Is_stop;
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
bool SetMap(int id){
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
void onRoadRecvd(const custom_msgs::LaneLineArray::ConstPtr &msg){
    ros_road_line = *msg;
}
void onCurPoseSDRecvd(const custom_msgs::CurPose::ConstPtr &msg){
    ros_cur_pose = *msg;
}
int get_LaneID_WithMutex(){
    int res;
    if(ros_road_line.lines.size()==0)
        return -1;
    res = ros_road_line.lines[0].current_lane_num;
    return res;
}
void onWarnCodeMsgRecvd(const custom_msgs::WarnCode::ConstPtr &msg)
{
    uint8_t lidar=0x00,navi=0x00,ctrl=0x00,xavier=0x00;
    if((*msg).lidar!=0)
    {
          lidar=0x01;
    }
    else if((*msg).navi!=0)
    {
         navi=0x02;
    }
    else if((*msg).ctrl!=0)
    {
         ctrl=0x04;
    }
    else if((*msg).xavier!=0)
    {
        xavier=0x08;
    }
    vichle_statu.exception_code=lidar+ navi+ctrl+xavier;
}
void onVehMsgRecvd(const custom_msgs::VehicleStat::ConstPtr &msg){
    //vichle_statu.battery_left   = (*msg).battery_left ;
    vichle_statu.speed   = (*msg).VehicleSpeed;
    if((*msg).ParkSts==1)
    {
        vichle_statu.vichle_status_code=0x03;
    }
    else if((*msg).GearShiftPositon==1)
    {
        vichle_statu.vichle_status_code=0x01;
    }
    else if((*msg).GearShiftPositon==-1)
    {
        vichle_statu.vichle_status_code=0x02;
    }
    else
    {
        vichle_statu.vichle_status_code=0x00;
    }
}
void sendthd() {
    ros::spin();
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "interactive_node");
    ros::NodeHandle nh;

    ros::Subscriber sub_navi = nh.subscribe("/navi_msg",1,onNaviMsgRecvd);//接受车辆定位
    ros::Subscriber sub_veh_statu   = nh.subscribe("/VehicleStat",1,onVehMsgRecvd);//接受车辆状态
    //ros::Subscriber sub_aeb_time   = nh.subscribe("/aeb_time",1,onAebtMsgRecvd);//接受aeb是否超时
    ros::Subscriber sub_warn_code   = nh.subscribe("/WarnCode",1,onWarnCodeMsgRecvd);   //接受异常代码
    ros::Subscriber sub_road         = nh.subscribe("/road_lane", 1,onRoadRecvd);//接受地图道路
    ros::Subscriber sub_cur_pose     = nh.subscribe("/cur_pose_all", 1 , onCurPoseSDRecvd);
    mapsw_client  = nh.serviceClient<custom_msgs::Map_Switch>("/map_switch_server"); //地图切换
    control_client   = nh.serviceClient<custom_msgs::Control>("/control_service");
    map_pub_client  = nh.serviceClient<custom_msgs::Task>("/task_exc_service");

    thread t(sendthd);
    t.detach();
    core.ipfigFileRead();
    core.socket_init();
    while(ros::ok()){
        int recv_num = 0;
        int send_num = 0;
        int time_num=0;
        uint16_t msg=0;
        map_id = get_LaneID_WithMutex();//获取当前地图段
        ROS_INFO_STREAM_THROTTLE(2,"map_id    :"<<map_id);
        ROS_INFO_STREAM_THROTTLE(2,"ros_cur_pose.s    :"<<ros_cur_pose.s);
        vichle_statu.timestamp = (ros::Time::now().toSec()) * 1000;
        ROS_INFO_STREAM_THROTTLE(1,"vichle_statu.direct: "<<vichle_statu.direct);
        ROS_INFO_STREAM_THROTTLE(1,"vichle_statu.lon: "<<vichle_statu.lon);
        ROS_INFO_STREAM_THROTTLE(1,"vichle_statu.lat : "<<vichle_statu.lat );
        ROS_INFO_STREAM_THROTTLE(1,"vichle_statu.speed: "<<vichle_statu.speed);
        ROS_INFO_STREAM_THROTTLE(1,"vichle_statu.timestamp: "<<vichle_statu.timestamp);
        ROS_INFO_STREAM_THROTTLE(1,"vichle_statu.battery_left : "<<vichle_statu.battery_left );
        ROS_INFO_STREAM_THROTTLE(1,"vichle_statu.vichle_status_code: "<<(int)vichle_statu.vichle_status_code);
        ROS_INFO_STREAM_THROTTLE(1,"vichle_statu.exception_code: "<<(int)vichle_statu.exception_code);
        ROS_INFO_STREAM_THROTTLE(1,"vichle_statu.route_code: "<<(int)vichle_statu.route_code);
        ROS_INFO_STREAM_THROTTLE(1,"vichle_statu.vichle_task_code: "<<(int)vichle_statu.vichle_task_code);
        ros::Rate loop_rate(10);//控制循环的频率，10hz
        send_num=core.send_data((char *)&vichle_statu,sizeof(vichle_statu), 0);
        ROS_INFO_STREAM_THROTTLE(1,"send_num: "<<send_num);
        if (send_num <= 0)
        {
            core.socket_close();
            core.socket_init();
        }
        memset(recv_buf,0,sizeof(recv_buf));
        recv_num = core.recv_data(recv_buf, sizeof(recv_buf), 0);
        if (recv_num > 0)
        {
            cout << "recv_num = " << recv_num << endl;

            memcpy((char *)&msg,recv_buf,sizeof(uint16_t));
            // BigAndLittleEndianConversion16(&task_send.header, sizeof(task_send.header));
            // BigAndLittleEndianConversion64(&task_send.car_ser, sizeof(task_send.car_ser));
            // BigAndLittleEndianConversion64(&task_send.send, sizeof(task_send.send));     
        }
        switch (msg){
            case 5002:
                memcpy((char*)&state_control, recv_buf, sizeof(state_control));
                break;
            case 5003:
                memcpy((char*)&task_send, recv_buf, sizeof(task_send));
                break;
            default:
                break;
        }
        //状态控制指令
        switch (state_control.vichle_control_code)
        {
        case 0x00:
            vichle_statu.vichle_task_code=0x00;
            vichle_statu.route_code=0x00;
            memset((char*)&task_send,0,sizeof(task_send));
            SetStop(true,9,"taks stop");
            break;
        case 0x01:
            SetStop(true,9,"taks stop"); 
            break;
        case 0x02:
            SetStop(false,9,"taks stop"); 
        default:
            break;
        }
        //任务线路派发
        if(task_send.msg>0)
        {
            cout<<"task_send.msg: "<<(int)task_send.msg<<endl;
            cout<<"task_send.route_code: "<<(int)task_send.route_code<<endl;
            if(vichle_statu.vichle_task_code==0)
            {
                if(task_send.route_code==0x01)
                {
                    vichle_statu.route_code=task_send.route_code;
                    task_res.status=0x02;
                    vichle_statu.vichle_task_code=0x01;
                    //map_switch(1);
                }
            }
            else
            {
                task_res.status=0x00;
            }
            cout<<"task_res.status: "<<(int)task_res.status<<endl;
        }
        task_res.route_code=task_send.route_code;
        if (recv_num > 0 && msg==5003)
        {
            core.send_data((char*)&task_res, sizeof(task_res), 0);
        }
    }

    return 0;
}


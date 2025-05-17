#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>


#include <ros/ros.h>
#include <ros/time.h>

#include "custom_msgs/Warn.h"
#include "custom_msgs/Control.h"
#include "custom_msgs/CurPose.h"
#include "custom_msgs/WarnCode.h"
using namespace std;

struct VEH_STAT{
    uint8_t lidar_is_ok;
    uint8_t navi_is_ok;
    uint8_t ctrl_is_ok;
    uint8_t xavier_is_ok;
    VEH_STAT():lidar_is_ok(0),navi_is_ok(0),ctrl_is_ok(0),xavier_is_ok(0){};
} veh_stat;

custom_msgs::CurPose cur_pose;

bool SetAEB(bool enable,float distance,std::string source,ros::ServiceClient control_client){
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

void onCurPoseSDRecvd(const custom_msgs::CurPose::ConstPtr &msg){ 
    cur_pose = *msg;
}

bool onServCallRecvd(custom_msgs::Warn::Request &req,custom_msgs::Warn::Response &res){
    switch (req.source){
        case 0:
            break;
        case 1:
            veh_stat.lidar_is_ok = req.code;
            break;
        case 2:
            veh_stat.navi_is_ok = req.code;
            break;
        case 3:
            veh_stat.ctrl_is_ok = req.code;
            break;
        
        default:
            break;
    }
    res.isLidarOK = veh_stat.lidar_is_ok;
    res.isNaviOK = veh_stat.navi_is_ok;
    res.isCtrlOK = veh_stat.ctrl_is_ok;
    res.isXavierOK = veh_stat.xavier_is_ok;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "warning_node");
    ros::NodeHandle nh;

    ros::ServiceServer warn_collect_service = nh.advertiseService("/warn_collect_service", onServCallRecvd);
    ros::ServiceClient control_client   = nh.serviceClient<custom_msgs::Control>("/control_service");
    ros::Subscriber    sub_cur_pose     = nh.subscribe("/cur_pose_all", 1 ,onCurPoseSDRecvd);
    ros::Publisher     pub_warn_code      = nh.advertise<custom_msgs::WarnCode>("WarnCode", 1);

    bool isHaveWarn = false;
    int err_time =0;
    while(ros::ok()){
        static custom_msgs::WarnCode warncode_msg;
        warncode_msg.lidar=veh_stat.lidar_is_ok;
        warncode_msg.navi=veh_stat.navi_is_ok;
        warncode_msg.ctrl=veh_stat.ctrl_is_ok;
        warncode_msg.xavier=veh_stat.xavier_is_ok;
        pub_warn_code.publish(warncode_msg);
        if(veh_stat.lidar_is_ok != 0 ||
            veh_stat.navi_is_ok != 0 ||
            veh_stat.ctrl_is_ok != 0 ||
            veh_stat.xavier_is_ok != 0){
            ROS_ERROR_STREAM_THROTTLE(1,"warn : "<<(int)veh_stat.lidar_is_ok <<" "<<(int)veh_stat.navi_is_ok <<" "<<(int)veh_stat.ctrl_is_ok <<" "<<(int)veh_stat.xavier_is_ok);
            err_time++;
            if(err_time>8){
                SetAEB(true,5.5,"warn_node",control_client);
            }  
            isHaveWarn = true;
        }else if(isHaveWarn == true){
            SetAEB(false,5.5,"warn_node",control_client);
            isHaveWarn = false;
            err_time = 0;
        }

        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }

    return 0;
}


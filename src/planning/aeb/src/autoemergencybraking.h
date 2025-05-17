#ifndef AUTOEMERGENCYBRAKING_H
#define AUTOEMERGENCYBRAKING_H

#include "ros/ros.h"
#include "iostream"
#include "custom_msgs/ObjectArray.h"
#include "custom_msgs/AEB.h"
#include <custom_msgs/LidarRawObjectArray.h>
#include <custom_msgs/RoadAttri.h>
#include <custom_msgs/VehicleStat.h>
#include <custom_msgs/Control.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <custom_msgs/CurPose.h>
#include <custom_msgs/NaviData.h>

typedef struct AEB_Range{
    double front;
    double back;
    double left;
    double right;
    AEB_Range():front(5.5),back(3.5),left(2.5),right(2.5){}
} AEB_Range;

class AutoEmergencyBraking
{
public:
    AutoEmergencyBraking();
    void exec(int argc, char **argv);

private:
    ros::NodeHandle nh;
    ros::Subscriber object_sub;
    ros::Subscriber attri_sub;
    ros::Subscriber vehicle_stat_sub;
    ros::Subscriber sub_cur_pose;
    ros::Subscriber sub_navi;
    tf2_ros::Buffer tf_buffer, tf_buffer_back;
    geometry_msgs::TransformStamped transform_stamped, transform_stamped_back;

    ros::ServiceClient control_client;
    bool SetAEB(bool enable,float distance,std::string source);

    void onObjectRecvd(const custom_msgs::ObjectArray::ConstPtr &msg);
    void onAttriRecvd(const custom_msgs::RoadAttri::ConstPtr &msg);
    void onVehicleStatRecvd(const custom_msgs::VehicleStat::ConstPtr &msg);   
    void onCurPoseSDRecvd(const custom_msgs::CurPose::ConstPtr &msg);
    void onNaviMsgRecvd(const custom_msgs::NaviData::ConstPtr &msg);
    

private:

    AEB_Range aebRange;
    bool is_GoFront;   //判断前进后退
    custom_msgs::CurPose cur_pose;
    custom_msgs::NaviData  navi_data;
    
    bool isThreat(const custom_msgs::Object &obj,float &dist);   
    bool isThreat_backcar(const custom_msgs::Object &obj, float &dist);  
};

#endif // AUTOEMERGENCYBRAKING_H

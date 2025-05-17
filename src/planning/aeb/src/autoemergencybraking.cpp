#include "autoemergencybraking.h"
#include <iostream>
using namespace std;


AutoEmergencyBraking::AutoEmergencyBraking()
{
    std::string object_topic;
    nh.param<std::string>("/aeb_node/object_topic", object_topic,"/ass_object");

    object_sub = nh.subscribe(object_topic,1,&AutoEmergencyBraking::onObjectRecvd,this);
    attri_sub  = nh.subscribe("/road_attri_msg", 1, &AutoEmergencyBraking::onAttriRecvd, this);
    vehicle_stat_sub  = nh.subscribe("/VehicleStat", 1, &AutoEmergencyBraking::onVehicleStatRecvd, this);
    control_client    = nh.serviceClient<custom_msgs::Control>("/control_service");
    sub_cur_pose     = nh.subscribe("/cur_pose_all", 1 , &AutoEmergencyBraking::onCurPoseSDRecvd, this);
    sub_navi         = nh.subscribe("/navi_msg",1,&AutoEmergencyBraking::onNaviMsgRecvd,this);

}

void AutoEmergencyBraking::exec(int argc, char **argv)
{
    tf2_ros::TransformListener tf_listener(tf_buffer);
    tf2_ros::TransformListener tf_listener_back(tf_buffer_back);

    ros::spin();
}

void AutoEmergencyBraking::onNaviMsgRecvd(const custom_msgs::NaviData::ConstPtr &msg){
    navi_data = *msg;
}

inline bool AutoEmergencyBraking::SetAEB(bool enable,float distance,std::string source){
    custom_msgs::Control srv;
    srv.request.type   = custom_msgs::Control::Request::Is_aeb;
    srv.request.enable = enable;
    srv.request.info   = distance;
    srv.request.source = source;
    if (control_client.call(srv)){
        return 1;
    }
    else{
        ROS_ERROR("AEB:Failed to call service contrl");
        return 0;
    }
}

void AutoEmergencyBraking::onCurPoseSDRecvd(const custom_msgs::CurPose::ConstPtr &msg){
    cur_pose = *msg;
}

void AutoEmergencyBraking::onAttriRecvd(const custom_msgs::RoadAttri::ConstPtr &msg){
    if(aebRange.front != msg->aeb_front
       || aebRange.back != msg->aeb_back
       || aebRange.left != msg->aeb_left
       || aebRange.right != msg->aeb_right){
        
        aebRange.front = msg->aeb_front;
        aebRange.back  = msg->aeb_back;
        aebRange.left  = msg->aeb_left;
        aebRange.right = msg->aeb_right;

        ROS_WARN_STREAM("AEB_Range is changed :" <<aebRange.front <<" "<<aebRange.back <<" "<<aebRange.left <<" "<<aebRange.right);
    }
}

void AutoEmergencyBraking::onVehicleStatRecvd(const custom_msgs::VehicleStat::ConstPtr &msg){
    if(msg->GearShiftPositon == -1)
        is_GoFront = false;
    else
        is_GoFront = true;
}

void AutoEmergencyBraking::onObjectRecvd(const custom_msgs::ObjectArray::ConstPtr &msg){
    try {
        transform_stamped = tf_buffer.lookupTransform("base_link", "map",
                                                      ros::Time(0)); 
        //transform_stamped_back = tf_buffer_back.lookupTransform("lidar42", "map",
                                                     // ros::Time(0));
    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
        return;
    }

    float min_dist=999,dist=999;
    if(is_GoFront){//前进情况下

        bool is_have_threat = false;
        for(const auto &var : msg->objs){
            //cout<<"1111"<<endl;
            if(isThreat(var,dist)){
                //cout<<"22222"<<endl;
                is_have_threat = true;
                min_dist = dist<min_dist ? dist : min_dist;
                SetAEB(true,min_dist,"AEB_node");
                 //ROS_WARN("%s","aeb");
                //std::cout<<"111111"<std::endl;
            }
        }
        SetAEB(is_have_threat, min_dist,"AEB_node");//如果不存在威胁目标，该句能取消aeb使能

    }else{//倒车情况下

        bool is_have_threat = false;
        for(const auto &var : msg->objs){
            if(isThreat_backcar(var,dist)){
                is_have_threat = true;
                min_dist = dist<min_dist ? dist : min_dist;
                if(cur_pose.s >1100 && cur_pose.s <1150){
                    is_have_threat = false;
                    continue;
                }
                if(navi_data.pose_type != 50){
                    is_have_threat = false;
                    continue;
                }
                SetAEB(true,min_dist,"AEB_node");
            }
        }
        SetAEB(is_have_threat, min_dist,"AEB_node");//如果不存在威胁目标，该句能取消aeb使能

    }
}    

bool AutoEmergencyBraking::isThreat(const custom_msgs::Object &obj, float &dist)  
{
    geometry_msgs::Point point;
    point.x = obj.x_pos;
    point.y = obj.y_pos;
    point.z = 0;
    
    tf2::doTransform(point,point,transform_stamped);
    if(point.y>0)
        point.y=point.y - obj.lwh.x/2;
    else
        point.y=point.y + obj.lwh.x/2;
    
    if(point.x > aebRange.right  || point.x < -aebRange.left) return false;
    if(point.y > aebRange.front || point.y < -aebRange.back)  return false;

    dist = point.y;
    return true;
}

bool AutoEmergencyBraking::isThreat_backcar(const custom_msgs::Object &obj, float &dist)   
{
    geometry_msgs::Point point;
    point.x = obj.x_pos;
    point.y = obj.y_pos;
    point.z = 0;
    
    tf2::doTransform(point,point,transform_stamped_back);

    if(point.y > 2.5 || point.y < -2.5)  return false;
    if(point.x > 1   || point.x < -0.55) return false;
   
    dist = 5.1;
    return true;
}

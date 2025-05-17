#ifndef HYBRID_ASTAR_TASK_H
#define HYBRID_ASTAR_TASK_H

#include <ros/ros.h>
#include <ros/time.h>
#include <stdlib.h>

#include <string>
#include <string.h>
#include <iostream>
#include<vector>
#include "datatype.h"
#include <custom_msgs/CurPose.h>
#include <custom_msgs/Task.h>
#include <custom_msgs/CarDirection.h>
#include <fstream>
#include <sstream>
#include <ros/package.h>
#include<custom_msgs/RoadAttri.h>
#include <thread>
#include <mutex>

using namespace std;


class HybridAstarTask
{
public:
    HybridAstarTask() {
        sub_cur_pose     = nh.subscribe("/cur_pose_all", 1 , &HybridAstarTask::onCurrentPoseMsgRecvd, this);
        sub_segment_msg     = nh.subscribe("/car_direction", 1 , &HybridAstarTask::onSegmentMsgRecvd, this);

        task_exc_client  = nh.serviceClient<custom_msgs::Task>("/task_exc_service");
        road_attri_pub = nh.advertise<custom_msgs::RoadAttri>("/road_attri_msg", 1);
        timer = nh.createTimer(ros::Duration(0.1), &HybridAstarTask::timerCallback, this);
    }
public:

    // void exec();                         //激活回调函数
    vector<TaskAttribute> task_atr_vec; //TaskAttribute集合
    vector<RoadAttribute> road_atr_vec;


    void getTaskS(custom_msgs::CarDirection msg);
    void hyAstarBackCarTask();
    void hyAstarStopCarTask();

    vector<RoadAttribute> readDataFromFile(const string& filename);
    void GetRoadAtr();
    // double pre_car_pose_s = -1;
    // double pre_stop_s = -1;
    double pre_task_id = -1;
    double pre_stop_id = -1;

private:
    struct TaskAttribute task_atr;      //读取TaskAttribute
	struct RoadAttribute road_atr;
    int res;
    custom_msgs::CarDirection segment_msg;
    custom_msgs::CurPose curpose_msg;
    custom_msgs::Task task; 

    ros::NodeHandle nh; 

    ros::Publisher road_attri_pub;
    //消息订阅器
    ros::Subscriber sub_cur_pose;
    ros::Subscriber sub_segment_msg;

    ros::Timer timer;

    

    //回调函数
    void onCurrentPoseMsgRecvd(const custom_msgs::CurPose::ConstPtr &msg);
    void onSegmentMsgRecvd(const custom_msgs::CarDirection::ConstPtr &msg);
    void timerCallback(const ros::TimerEvent& evnt);

    ros::ServiceClient task_exc_client;

private:
    std::mutex               mtx_car_direction;
    custom_msgs::CarDirection    get_CarDirection_WithMutex();
};


  

#endif // 


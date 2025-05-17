#pragma once

#include <ros/ros.h>

#include <pix_robot_can/MotorInfo.h>
#include <pix_robot_can/SonarData.h>
#include <pix_robot_can/SystemStatus.h>

#include "vcu_message.hpp"
#include <geometry_msgs/Twist.h>

class TopicPublish
{
public:
    TopicPublish(std::shared_ptr<ros::NodeHandle> pNodeHandle);

    void Publish(pix_robot_can::MotorInfo);
    void Publish(pix_robot_can::SonarData);
    void Publish(pix_robot_can::SystemStatus);

protected:
    // Pointers to publishers
    std::shared_ptr<ros::Publisher> mpPubMotorInfo;
    std::shared_ptr<ros::Publisher> mpPubSystemStatus;
    std::shared_ptr<ros::Publisher> mpPubSonarData;
};

class MyTopicPublish
{
public:
    MyTopicPublish(std::shared_ptr<ros::NodeHandle> pNodeHandle);

    void Publish(geometry_msgs::Twist);

protected:
    // Pointers to publishers
    std::shared_ptr<ros::Publisher> pix_com_vel;
};



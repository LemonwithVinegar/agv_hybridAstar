
#include "topic_publish.hpp"

TopicPublish::TopicPublish(std::shared_ptr<ros::NodeHandle> pNodeHandle)
{
    // Topics need to be published.

    mpPubMotorInfo = std::make_shared<ros::Publisher>(pNodeHandle->advertise<pix_robot_can::MotorInfo>("pix/motor_info", 1));
    mpPubSystemStatus = std::make_shared<ros::Publisher>(pNodeHandle->advertise<pix_robot_can::SystemStatus>("pix/system_status", 1));
    mpPubSonarData = std::make_shared<ros::Publisher>(pNodeHandle->advertise<pix_robot_can::SonarData>("pix/sonar_data", 1));
}

MyTopicPublish::MyTopicPublish(std::shared_ptr<ros::NodeHandle> pNodeHandle)
{
    pix_com_vel = std::make_shared<ros::Publisher>(pNodeHandle->advertise<geometry_msgs::Twist>("pix/com_vel", 1));
}

void TopicPublish::Publish(pix_robot_can::MotorInfo motorInfo)
{
    mpPubMotorInfo->publish(motorInfo);
}

void TopicPublish::Publish(pix_robot_can::SonarData sonarData)
{
    mpPubSonarData->publish(sonarData);
}

void TopicPublish::Publish(pix_robot_can::SystemStatus systemStatus)
{
    //systemStatus.manualMode = 1;
    mpPubSystemStatus->publish(systemStatus);
}

void MyTopicPublish::Publish(geometry_msgs::Twist com_vel)
{
    pix_com_vel->publish(com_vel);
}
#include <ros/ros.h>

#include "message_callbacks.hpp"
#include "vcu_communicate.hpp"



static geometry_msgs::Twist com_vel;

void TopicCallbacks::SetMotorSpeedCallback(const  pix_robot_can::SetMotorSpeed::ConstPtr &msg)
{
    int32_t speed[MOTOR_TOTAL_NUM];
    for(int i = 0; i < MOTOR_TOTAL_NUM; ++i) speed[i] = msg->motorSpeed[i];
    VcuCommunicate::GetInstance().SetMotorSpeed(speed);
//    ROS_INFO("Set motor speed.");
}

void TopicCallbacks::SetMotorParametersCallback(const pix_robot_can::SetMotorParameters::ConstPtr &msg)
{
    VcuCommunicate::GetInstance().SetMotorParameters(msg->maxSpeed, msg->accTime, msg->decTime);
//    ROS_INFO("Set motor parameters.");
}

void TopicCallbacks::ParkingBrakeCallback(const pix_robot_can::ParkingBrake::ConstPtr &msg)
{
    VcuCommunicate::GetInstance().ParkingBrake(msg->ifBrake);
//    ROS_INFO("Parking brake.");
}

void TopicCallbacks::LightControlCallback(const pix_robot_can::LightControl::ConstPtr &msg)
{
    VcuCommunicate::GetInstance().LightControl(msg->turnSignal, msg->chassisLights, msg->headLights, msg->flagLight);
//    ROS_INFO("Light control.");
}

void TopicCallbacks::CargoUnlockCallback(const pix_robot_can::CargoUnlock::ConstPtr &msg)
{
    VcuCommunicate::GetInstance().CargoUnlock(msg->unlock);
//    ROS_INFO("Cargo unlock.");
}

void TopicCallbacks::DisplayOnLcdCallback(const pix_robot_can::DisplayOnLcd::ConstPtr &msg)
{
    uint8_t firstLine[16], secondLine[16];
    for(int i = 0; i < 16; ++i)
    {
        firstLine[i] = msg->firstLine[i];
        secondLine[i] = msg->secondLine[i];
    }
    VcuCommunicate::GetInstance().DisplayOnLcd(firstLine, secondLine);
//    ROS_INFO("Display on LCD.");
}

void TopicCallbacks::ResetMotorCallback(const pix_robot_can::ResetMotor::ConstPtr &msg)
{
    VcuCommunicate::GetInstance().ResetMotor(msg->motorId);
}

void TopicCallbacks::ImpactProtectCallback(const pix_robot_can::ImpactProtect::ConstPtr &msg)
{
    VcuCommunicate::GetInstance().ImpactProtect(msg->enableFrontRear, msg->enableSide);
}

void TopicCallbacks::SetRobotMotionCallback(const pix_robot_can::SetRobotMotion::ConstPtr &msg)
{
    VcuCommunicate::GetInstance().SetRobotMotion(msg->velocity, msg->angularVelocity);
}

void TopicCallbacks::GeometryTwistCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    VcuCommunicate::GetInstance().SetRobotMotion(msg->linear.x, msg->angular.z);
    #ifdef MESSAGECALLBACK_DEBUG
    ROS_INFO("pix/com_vel recived");
    ROS_INFO("geometry_msgs::Twist msg->linear.x = %f", msg->linear.x);
    ROS_INFO("geometry_msgs::Twist msg->angular.z = %f", msg->angular.z);
    #endif
}

void MyTopicCallbacks::LatGeometryTwistCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    com_vel.angular.z = msg->angular.z;
    VcuCommunicate::GetInstance().SetRobotMotion(com_vel.linear.x, com_vel.angular.z);
    #ifdef MESSAGECALLBACK_DEBUG
    ROS_INFO("pix/com_vel recived");
    ROS_INFO("geometry_msgs::Twist msg->linear.x = %f", com_vel.linear.x);
    ROS_INFO("geometry_msgs::Twist msg->angular.z = %f", com_vel.angular.z);
    #endif
}
void MyTopicCallbacks::LonGeometryTwistCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    com_vel.linear.x = msg->linear.x;
    VcuCommunicate::GetInstance().SetRobotMotion(com_vel.linear.x, com_vel.angular.z);
    #ifdef MESSAGECALLBACK_DEBUG
    ROS_INFO("pix/com_vel recived");
    ROS_INFO("geometry_msgs::Twist msg->linear.x = %f", com_vel.linear.x);
    ROS_INFO("geometry_msgs::Twist msg->angular.z = %f", com_vel.angular.z);
    #endif
}

void TopicCallbacks::SetEmergencyCallback(const pix_robot_can::SetEmergencyStatus::ConstPtr &msg)
{
    VcuCommunicate::GetInstance().SetEmergency(msg->emergencyStatus);
}

void TopicCallbacks::SetAckermannMotionCallback(const pix_robot_can::SetAckermannMotion::ConstPtr &msg)
{
    VcuCommunicate::GetInstance().SetAckermannMotion(msg->velocity, msg->ackermannAngle);
}

void TopicCallbacks::SetSteeringModeCallback(const pix_robot_can::SetSteeringMode::ConstPtr &msg)
{
    VcuCommunicate::GetInstance().SetSteeringMode(msg->steeringMode);
}

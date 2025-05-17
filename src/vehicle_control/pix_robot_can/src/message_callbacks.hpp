#pragma once

#include <pix_robot_can/SetMotorSpeed.h>
#include <pix_robot_can/SetMotorParameters.h>
#include <pix_robot_can/ParkingBrake.h>
#include <pix_robot_can/LightControl.h>
#include <pix_robot_can/CargoUnlock.h>
#include <pix_robot_can/DisplayOnLcd.h>
#include <pix_robot_can/ResetMotor.h>
#include <pix_robot_can/ImpactProtect.h>
#include <pix_robot_can/SetRobotMotion.h>
#include <geometry_msgs/Twist.h>
#include <pix_robot_can/SetEmergencyStatus.h>
#include <pix_robot_can/SetAckermannMotion.h>
#include <pix_robot_can/SetSteeringMode.h>

// #define MESSAGECALLBACK_DEBUG

class TopicCallbacks
{
public:
    void SetMotorSpeedCallback(const pix_robot_can::SetMotorSpeed::ConstPtr &msg);
    void SetMotorParametersCallback(const pix_robot_can::SetMotorParameters::ConstPtr &msg);
    void ParkingBrakeCallback(const pix_robot_can::ParkingBrake::ConstPtr &msg);
    void LightControlCallback(const pix_robot_can::LightControl::ConstPtr &msg);
    void CargoUnlockCallback(const pix_robot_can::CargoUnlock::ConstPtr &msg);
    void DisplayOnLcdCallback(const pix_robot_can::DisplayOnLcd::ConstPtr &msg);
    void ResetMotorCallback(const pix_robot_can::ResetMotor::ConstPtr &msg);
    void ImpactProtectCallback(const pix_robot_can::ImpactProtect::ConstPtr &msg);
    void SetRobotMotionCallback(const pix_robot_can::SetRobotMotion::ConstPtr &msg);
    void SetAckermannMotionCallback(const pix_robot_can::SetAckermannMotion::ConstPtr &msg);
    void GeometryTwistCallback(const geometry_msgs::Twist::ConstPtr &msg);
    void SetEmergencyCallback(const pix_robot_can::SetEmergencyStatus::ConstPtr &msg);
    void SetSteeringModeCallback(const pix_robot_can::SetSteeringMode::ConstPtr &msg);
};

class MyTopicCallbacks
{
public:
    //自定义的中转指令回调
    void LatGeometryTwistCallback(const geometry_msgs::Twist::ConstPtr &msg);
    void LonGeometryTwistCallback(const geometry_msgs::Twist::ConstPtr &msg);
};

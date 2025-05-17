#include <ros/ros.h>
#include <cmath>

#include "vcu_communicate.hpp"
#include "topic_publish.hpp"
#include "udp_interface.hpp"

VcuCommunicate VcuCommunicate::instance;
extern std::shared_ptr<TopicPublish> pTopicPublish;

VcuCommunicate::VcuCommunicate()
{
}

/**
 * @brief Thread of the communication to RCU
 * Receive messages from RCU and forward them to topics.
 * @param None
 * @retval None
 */
void VcuCommunicate::Task(void)
{
    ROS_INFO("VcuCommunicate::Task() started.");

    while (true)
    {
        uint32_t msg = mMsgQ.Pop();
        if (!msg)
            break;
        switch (msg)
        {
        case UDP_MSG_TYP_REPORT_MOTOR_INFO:
            while (!mMotorInfoQ.empty())
            {
                UdpMotorInfo_t motorInfo = mMotorInfoQ.front();
                mMotorInfoQ.pop();
                pix_robot_can::MotorInfo msgMotorInfo;
                msgMotorInfo.header.stamp = ros::Time::now();
                msgMotorInfo.header.frame_id = "motor_information";
                for (int i = 0; i < MOTOR_TOTAL_NUM; ++i)
                {
                    msgMotorInfo.motorStatus[i].runningStatus = motorInfo.runningStatus[i];
                    msgMotorInfo.motorStatus[i].speed = motorInfo.speed[i];
                    msgMotorInfo.motorStatus[i].position = motorInfo.position[i];
                    msgMotorInfo.motorStatus[i].temperature = motorInfo.temperature[i];
                }
                pTopicPublish->Publish(msgMotorInfo);
            }
            break;

        case UDP_MSG_TYP_REPORT_SONAR_DATA:
            while (!mSonarDataQ.empty())
            {
                UdpSonarData_t sonarData = mSonarDataQ.front();
                mSonarDataQ.pop();
                pix_robot_can::SonarData msgSonarData;
                for (int i = 0; i < SONAR_TOTAL_NUM; ++i)
                    msgSonarData.distance[i] = sonarData.distance[i];
                pTopicPublish->Publish(msgSonarData);
            }
            break;

        case UDP_MSG_TYP_REPORT_SYS_STATUS:
            while (!mSystemStatusQ.empty())
            {
                UdpSystemStatus_t systemStatus = mSystemStatusQ.front();
                mSystemStatusQ.pop();
                pix_robot_can::SystemStatus msgSystemStatus;
                msgSystemStatus.header.stamp = ros::Time::now();
                msgSystemStatus.header.frame_id = "system_status";
                msgSystemStatus.upTime = systemStatus.upTime;
                msgSystemStatus.errorCode = systemStatus.errorCode;
                msgSystemStatus.brakeStatus = systemStatus.brakeStatus;
                msgSystemStatus.temperature = systemStatus.temperature / 10;
                msgSystemStatus.cargoLock = systemStatus.cargoLock;
                msgSystemStatus.batteryStatus.voltage = systemStatus.voltage;
                msgSystemStatus.batteryStatus.current = systemStatus.current;
                msgSystemStatus.batteryStatus.capacity = systemStatus.capacity;
                msgSystemStatus.batteryStatus.inCharge = systemStatus.inCharge;
                msgSystemStatus.eStopStatus.frontEmStop = systemStatus.frontEmStop;
                msgSystemStatus.eStopStatus.rearEmStop = systemStatus.rearEmStop;
                msgSystemStatus.eStopStatus.remoteEmStop = systemStatus.remoteEmStop;
                msgSystemStatus.eStopStatus.rosEmStop = systemStatus.rosEmStop;
                msgSystemStatus.sonicImpact.frontRearImpacted = systemStatus.frontRearImpacted;
                msgSystemStatus.sonicImpact.sideImpacted = systemStatus.sideImpacted;
                msgSystemStatus.sonicImpact.impactBitmap = systemStatus.impactBitmap;
                msgSystemStatus.manualMode = systemStatus.manualMode;
                pTopicPublish->Publish(msgSystemStatus);
            }
            break;
        }
    }
    ROS_INFO("VcuCommunicate::Task() quit.");
}

/**
 * @brief Create and start communication thread.
 * @param None
 * @retval None
 */
void VcuCommunicate::StartThread(void)
{
    mThread = std::thread(std::bind(&VcuCommunicate::Task, this));
}

/**
 * @brief Stop the communication thread.
 * Stop the communication thread and wait it to quit.
 * @param None
 * @retval None
 */
void VcuCommunicate::StopThread(void)
{
    mMsgQ.Push(0);
    mThread.join();
}

/**
 * @brief Send heartbeat signal to RCU
 * @param None
 * @retval None
 */
void VcuCommunicate::SendHeartbeat(void)
{
    UdpHeartbeat_t data;
    data.msgType = UDP_MSG_TYP_HEARTBEAT;
    UdpInterface::GetInstance().SendMessageToVcu((uint8_t*)&data, sizeof(data));
}

/**
 * @brief Set speed of all hub motor one time.
 * @param speed Pointer to an array of the motor's speed in RPM.
 * @retval None
 */
void VcuCommunicate::SetMotorSpeed(int32_t* speed)
{
    UdpSetMotorSpeed_t data;
    data.msgType = UDP_MSG_TYP_SET_MOTOR_SPEED;
    for(int i = 0; i < MOTOR_TOTAL_NUM; ++i) data.motorSpeed[i] = speed[i];
    UdpInterface::GetInstance().SendMessageToVcu((uint8_t*)&data, sizeof(data));
}

/**
 * @brief Set hub motor's parameters.
 * Including maximum speed, accelerate time, decelerate time.
 * @param maxSpeed Maximum speed of motors.
 * @param accTime Accelerate time of motors.
 * @param decTime Decelerate time of motors.
 * @retval None
 */
void VcuCommunicate::SetMotorParameters(int32_t maxSpeed, int32_t accTime, int32_t decTime)
{
    UdpSetMotorParameters_t data;
    data.msgType = UDP_MSG_TYP_SET_MOTOR_PARAMETERS;
    data.maxSpeed = maxSpeed;
    data.accTime = accTime;
    data.decTime = decTime;
    UdpInterface::GetInstance().SendMessageToVcu((uint8_t*)&data, sizeof(data));
}

/**
 * @brief Parking brake control.
 * @param ifBrake True: brake False: release
 * @retval None
 */
void VcuCommunicate::ParkingBrake(bool ifBrake)
{
    UdpControlParkingBrake_t data;
    data.msgType = UDP_MSG_TYP_CONTROL_PARKING_BRAKE;
    data.ifBrake = ifBrake;
    UdpInterface::GetInstance().SendMessageToVcu((uint8_t*)&data, sizeof(data));
}

/**
 * @brief Control robot's lights
 * @param turnSignal Turning signal. -1 : Left 0 : Off 1 : Right
 * @param chassisLights Light on chassis ON/OFF
 * @param headLight Light at the front of the robot
 * @param flagLight Light on the flag. 0 : OFF; 1 : WHITE; 2 : RED; 3 : GREEN;
 * @retval None
 */
void VcuCommunicate::LightControl(int16_t turnSignal, bool chassisLights, bool headLights, uint32_t flagLight)
{
    UdpControlLight_t data;
    data.msgType = UDP_MSG_TYP_CONTROL_LIGHTS;
    data.turnSignal = turnSignal;
    data.chassisLights = chassisLights;
    data.headLights = headLights;
    data.flagLight = flagLight;
    UdpInterface::GetInstance().SendMessageToVcu((uint8_t*)&data, sizeof(data));
}

/**
 * @brief Unlock the cargo lid.
 * @param unlock True : unlock; False : do nothing;
 * @retval None
 */
void VcuCommunicate::CargoUnlock(bool unlock)
{
    UdpControlCargoUnlock_t data;
    data.msgType = UDP_MSG_TYP_CONTROL_CARGO_UNLOCK;
    data.ifUnlock = unlock;
    UdpInterface::GetInstance().SendMessageToVcu((uint8_t*)&data, sizeof(data));
}

/**
 * @brief Display characters on LCD
 * @param firstLine Pointer to the first line characters.
 * @param secondLine Pointer to the second line characters.
 * @retval None
 */
void VcuCommunicate::DisplayOnLcd(uint8_t* firstLine, uint8_t* secondLine)
{
    UdpDisplayOnLcd_t displayLcd;
    displayLcd.msgType = UDP_MSG_TYP_DISPLAY_ON_LCD;
    memcpy(displayLcd.firstLine, firstLine, sizeof(displayLcd.firstLine));
    memcpy(displayLcd.secondLine, secondLine, sizeof(displayLcd.secondLine));
    UdpInterface::GetInstance().SendMessageToVcu((uint8_t*)&displayLcd, sizeof(displayLcd));
}

/**
 * @brief Reset a hub motor
 * @param motorId Hub motor ID , from 0-3
 * @retval None
 */
void VcuCommunicate::ResetMotor(int32_t motorId)
{
    UdpResetMotor_t msg;
    msg.motorId = motorId;
    UdpInterface::GetInstance().SendMessageToVcu((uint8_t*)&msg, sizeof(msg));
}

/**
 * @brief Enable / disable sonar protections.
 * @param enableFrontRear Enable/Disable sonar protection at front and rear.
 * @param enableSide Enable/Disable sonar protection at chassis both sides.
 * @retval None
 */
void VcuCommunicate::ImpactProtect(bool enableFrontRear, bool enableSide)
{
    UdpImpactProtect_t msg;
    msg.enableFrontRear = enableFrontRear;
    msg.enableSide = enableSide;
    UdpInterface::GetInstance().SendMessageToVcu((uint8_t*)&msg, sizeof(msg));
}

void VcuCommunicate::CargoLid(uint32_t action)
{
    UdpOpenCargoLid_t msg;
    msg.action = action;
    UdpInterface::GetInstance().SendMessageToVcu((uint8_t*)&msg, sizeof(msg));
}

void VcuCommunicate::SetRobotMaxSpeed(uint32_t maxSpeed)
{
    UdpSetRobotMaxSpeed_t msg;
    msg.maxSpeed = maxSpeed;
    UdpInterface::GetInstance().SendMessageToVcu((uint8_t*)&msg, sizeof(msg));
}

void VcuCommunicate::SetRobotMotion(float velocity, float angularVelocity)
{
    UdpSetRobotMotion_t msg;
    msg.velocity = velocity;
    msg.angularVelocity = angularVelocity;
    UdpInterface::GetInstance().SendMessageToVcu((uint8_t*)&msg, sizeof(msg));
}

void VcuCommunicate::SetAckermannMotion(float velocity, float ackermannAngle)
{
    UdpSetAckermannAngle_t msg;
    msg.velocity = velocity;
    msg.ackermannAngle = ackermannAngle;
    UdpInterface::GetInstance().SendMessageToVcu((uint8_t*)&msg, sizeof(msg));
}

void VcuCommunicate::SetSteeringMode(uint32_t steeringMode)
{
    UdpSetSteeringMode_t msg;
    msg.steeringMode = steeringMode;
    UdpInterface::GetInstance().SendMessageToVcu((uint8_t*)&msg, sizeof(msg));
}

void VcuCommunicate::SetEmergency(bool emergencyStatus)
{
    UdpSetEmergency_t msg;
    msg.emergencyStatus = emergencyStatus;
    UdpInterface::GetInstance().SendMessageToVcu((uint8_t*)&msg, sizeof(msg));
}

/**
 * @brief Push a message to motor info message queue.
 * @param motorInfo Reference to a message of UdpMotorInfo
 * @retval None
 */
void VcuCommunicate::PushMessage(UdpMotorInfo_t &motorInfo)
{
    mMotorInfoQ.push(motorInfo);
    mMsgQ.Push(UDP_MSG_TYP_REPORT_MOTOR_INFO);
}

/**
 * @brief Push a message to sonar data message queue.
 * @param sonarData Reference to a message of UdpSonarData
 * @retval None
 */
void VcuCommunicate::PushMessage(UdpSonarData_t &sonarData)
{
    mSonarDataQ.push(sonarData);
    mMsgQ.Push(UDP_MSG_TYP_REPORT_SONAR_DATA);
}

/**
 * @brief Push a message to system status message queue.
 * @param sysStatus Reference to a message of UdpSystemstatus
 * @retval None
 */
void VcuCommunicate::PushMessage(UdpSystemStatus_t &sysStatus)
{
    mSystemStatusQ.push(sysStatus);
    mMsgQ.Push(UDP_MSG_TYP_REPORT_SYS_STATUS);
}

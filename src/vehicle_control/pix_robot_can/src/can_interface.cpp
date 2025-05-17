#include <iostream>
#include <functional>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include <ros/ros.h>

#include "can_interface.hpp"

CanInterface::CanInterface()
    : mThread(std::bind(&CanInterface::Task, this))
    , mQuit(false)
{
}

CanInterface::~CanInterface()
{
}

void CanInterface::Task(void)
{ 
    ROS_INFO("CAN interface thread started.");
    mSocket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if(mSocket < 0)
    {
        ROS_ERROR("Can't open a socket for CAN, quit.");
        exit(EXIT_FAILURE);
    }

    struct ifreq ifr;
    strcpy(ifr.ifr_name, "can0");
    ioctl(mSocket, SIOCGIFINDEX, &ifr);

    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    // Bind socket address
    if(bind(mSocket, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        ROS_ERROR("Bind CAN address fail, quit.");
        ROS_ERROR("Check if the CAN device is installed and using command \"sudo ip link set can0 up type can bitrate 500000\" to make it online.");
        close(mSocket);
        exit(EXIT_FAILURE);
    }

    // Set filter
    struct can_filter filter[2];
    filter[0].can_id = COB_ID_MOTOR_2_VCU;
    filter[0].can_mask = 0xFF8;
    filter[1].can_id = COB_ID_REMOTE_2_VCU;
    filter[1].can_mask = 0xFFF;
    setsockopt(mSocket, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter));

    struct can_frame frame;
    struct CanFrameMotor4 motorData;

    while(!mQuit.load())
    {
        int nbytes = read(mSocket, &frame, sizeof(struct can_frame));
        if(frame.can_dlc != 8) continue;

        if(COB_ID_MOTOR_2_VCU == (frame.can_id & 0x0FF8))
        {
            memcpy(&motorData, frame.data, 8);
            int motorId = (frame.can_id & 0x07) - 1;
            if(motorId < 0 || motorId > MOTOR_NUM) continue;

            switch(motorData.frameType)
            {
                case CAN_MOTOR_DATA_POSITION:
                {
                    int position = motorData.data;
                    std::lock_guard<std::mutex> lock(mPositionMutex);
                    if(motorId % 2) mMotorPosition[motorId] = -position;
                    else mMotorPosition[motorId] = position;
                }
                break;

                case CAN_MOTOR_DATA_SPEED:
                {
                    int speed = motorData.data;
                    std::lock_guard<std::mutex> lock(mSpeedMutex);
                    if(motorId % 2) mMotorSpeed[motorId] = -speed;
                    else mMotorSpeed[motorId] = speed;
                }
                break;
            }
        }
        else if(COB_ID_REMOTE_2_VCU == frame.can_id)
        {
        }
    }

    close(mSocket);
    ROS_INFO("CAN interface thread quited.");
}

/**
 * \brief Stop the thread and let it quit.
 * \param none
 * \return none
 */
void CanInterface::StopThread(void)
{
    mQuit = true;
    mThread.join();
}

/**
 * \brief Get current motor speed by motor id.
 * This method provide an interface to other programs to get current motor speed.
 * \param motorId motor id 0-3
 * \return Speed of motor of motorId
 */
int CanInterface::ReadMotorSpeed(int motorId)
{
    try
    {
        if(motorId < 0 || motorId > MOTOR_NUM) throw("Motor ID isn't in legal range.");
        std::lock_guard<std::mutex> lock(mSpeedMutex);
        return mMotorSpeed[motorId];
    }
    catch(const std::exception& e)
    {
        ROS_ERROR("%s", e.what());
    }
}

/**
 * \brief Get current motor encoder position by motor id.
 * This method provide an interface to other programs to get current motor encoder position.
 * \param motorId motor id 0-3
 * \return Encoder position of motor of motorId
 */
int CanInterface::ReadMotorEncoder(int motorId)
{
    try
    {
        if(motorId < 0 || motorId > MOTOR_NUM) throw("Motor ID isn't in legal range.");
        std::lock_guard<std::mutex> lock(mPositionMutex);
        return mMotorPosition[motorId];
    }
    catch(const std::exception& e)
    {
        ROS_ERROR("%s", e.what());
    }
}

bool CanInterface::WriteMotorSpeed(uint16_t motorId, int32_t rpm)
{
    struct can_frame frame;
    frame.can_id = COB_ID_VCU_2_MOTOR | (motorId + 1);
    frame.can_dlc = 8;
    struct CanFrameMotor4 motorData;
    motorData.dataLen = 23;
    motorData.frameType = CAN_MOTOR_COMMAND_SPEED;
    motorData.data = rpm;
    int nBytes = write(mSocket, &frame, sizeof(frame));
    if(8 == nBytes) return true;
    else return false;
}

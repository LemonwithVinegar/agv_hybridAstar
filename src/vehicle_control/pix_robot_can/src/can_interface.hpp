#pragma once

#include <mutex>
#include <thread>
#include <atomic>

class CanInterface
{
public:
    CanInterface();
    ~CanInterface();
    void StopThread();
    int ReadMotorSpeed(int motorId);
    int ReadMotorEncoder(int motorId);
    bool WriteMotorSpeed(uint16_t motorId, int32_t rpm);

protected:
    void Task();

private:
    std::mutex mPositionMutex;
    std::mutex mSpeedMutex;

protected:
    static const uint32_t COB_ID_VCU_2_MOTOR = 0x600;
    static const uint32_t COB_ID_MOTOR_2_VCU = 0x580;
    static const uint32_t COB_ID_REMOTE_2_VCU = 0x283;
    static const uint32_t COB_ID_VCU_2_REMOTE = 0x190;

    static const uint16_t CAN_MOTOR_DATA_POSITION = 0x6064;
    static const uint16_t CAN_MOTOR_DATA_SPEED = 0x606C;
    static const uint16_t CAN_MOTOR_COMMAND_SPEED = 0x60FF;

    std::thread mThread;
    std::atomic<bool> mQuit;

    int mSocket;

#define MOTOR_NUM 4
    int mMotorPosition[MOTOR_NUM];
    int mMotorSpeed[MOTOR_NUM];
    int mMotorGoalSpeed[MOTOR_NUM];

protected:
    struct CanFrameMotor4
    {
        uint8_t dataLen;
        uint16_t frameType;
        uint8_t dummy;
        uint32_t data;
    }__attribute__ ((packed));
};

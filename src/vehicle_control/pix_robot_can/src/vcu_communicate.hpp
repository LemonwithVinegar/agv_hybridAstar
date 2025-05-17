#pragma once

#include <queue>
#include <thread>
#include <atomic>
#include "message_queue.hpp"
#include "vcu_message.hpp"

class VcuCommunicate
{
private:
    VcuCommunicate();
    void operator =(const VcuCommunicate&) = delete;
    VcuCommunicate(const VcuCommunicate& ) = delete;

public:
    void StartThread();
    void StopThread();

    void SendHeartbeat();

    void SetMotorSpeed(int32_t* speed);
    void SetMotorParameters(int32_t maxSpeed, int32_t accTime, int32_t decTime);
    void ResetMotor(int32_t motorId);

    void ParkingBrake(bool ifBrake);
    void LightControl(int16_t turnSignal, bool chassisLights, bool headLights, uint32_t flagLight);
    void CargoUnlock(bool unLock);
    void DisplayOnLcd(uint8_t* firstLine, uint8_t* secondLine);
    void ImpactProtect(bool enableFrontRear, bool enableSide);
    void CargoLid(uint32_t action);
    void SetRobotMaxSpeed(uint32_t maxSpeed);
    void SetRobotMotion(float velocity, float angularVelocity);
    void SetEmergency(bool emergencyStatus);
    void SetAckermannMotion(float velocity, float ackermannAngle);
    void SetSteeringMode(uint32_t steeringMode);
    
    void PushMessage(UdpMotorInfo_t &motorInfo);
    void PushMessage(UdpSonarData_t &sonarData);
    void PushMessage(UdpSystemStatus_t &sysStatus);

    static VcuCommunicate& GetInstance() { return instance; }

protected:
    void Task();

private:
    static VcuCommunicate instance;

protected:
    std::thread mThread;
    MessageQueue<uint32_t> mMsgQ;

    std::queue<UdpMotorInfo_t> mMotorInfoQ;
    std::queue<UdpSonarData_t> mSonarDataQ;
    std::queue<UdpSystemStatus_t> mSystemStatusQ;
};

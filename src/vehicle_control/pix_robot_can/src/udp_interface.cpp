#include "udp_interface.hpp"
#include <functional>
#include <ros/ros.h>
#include <errno.h>
#include <sys/time.h>

UdpInterface UdpInterface::instance;

UdpInterface::UdpInterface()
{
    memset(&mVcuAddr, 0, sizeof(mVcuAddr));
    mVcuAddr.sin_family = AF_INET;
    mVcuAddr.sin_addr.s_addr = inet_addr(DEFAULT_VCU_IP);
    mVcuAddr.sin_port = htons(DEFAULT_VCU_IP_VCU_UDP_PORT);
}

void UdpInterface::Task(void)
{
    ROS_INFO("UdpInterface::Task() started.");
    char str[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &(mVcuAddr.sin_addr), str, INET_ADDRSTRLEN);
    ROS_INFO("RCU's IP = %s, PORT = %d.", str, ntohs(mVcuAddr.sin_port));

    try
    {
        mQuit = false;
        mSocket = socket(AF_INET, SOCK_DGRAM, 0);
        if (mSocket < 0)
            throw std::runtime_error("Can't open socket! Quit!");

        // Set UDP receive wait time out, after 100ms
        struct timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 100000;
        int result = setsockopt(mSocket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
        if (result < 0)
            throw std::runtime_error("Can't set sockopt! Quit!");
    }
    catch (const std::runtime_error &e)
    {
        ROS_ERROR("%s", e.what());
        exit(EXIT_FAILURE);
    }

    ROS_INFO("UdpInterface::Task() Initialize network successful.");

    while (!mQuit.load())
    {
        socklen_t len;
        int recvdLen = recvfrom(mSocket, mRecvMsg, RECV_MSG_BUF_LEN, 0, (struct sockaddr *)&mVcuAddr, &len);
        if (recvdLen < 0 && errno != 11)
        {
            ROS_ERROR("UdpInterface::Task() recvfrom() error. Error no : %d, error : %s", errno, strerror(errno));
            continue;
        }

        uint32_t messageType;
        memcpy(&messageType, mRecvMsg, sizeof(uint32_t));
        if (UDP_MSG_TYP_REPORT_SYS_STATUS == messageType)
        {
            if (recvdLen != sizeof(UdpSystemStatus_t))
                continue;
            UdpSystemStatus_t systemStatus;
            memcpy(&systemStatus, mRecvMsg, sizeof(UdpSystemStatus_t));
            VcuCommunicate::GetInstance().PushMessage(systemStatus);
        }
        else if (UDP_MSG_TYP_REPORT_MOTOR_INFO == messageType)
        {
            if (recvdLen != sizeof(UdpMotorInfo_t))
                continue;
            UdpMotorInfo_t motorInfo;
            memcpy(&motorInfo, mRecvMsg, sizeof(UdpMotorInfo_t));
            VcuCommunicate::GetInstance().PushMessage(motorInfo);
        }
        else if (UDP_MSG_TYP_REPORT_SONAR_DATA == messageType)
        {
            if (recvdLen != sizeof(UdpSonarData_t))
                continue;
            UdpSonarData_t sonarData;
            memcpy(&sonarData, mRecvMsg, sizeof(UdpSonarData_t));
            VcuCommunicate::GetInstance().PushMessage(sonarData);
        }
    }
    ROS_INFO("UdpInterface::Task() quit.");
}

/**
 * @brief Create and start thread of UDP interface.
 * @param None
 * @retval None
 */
void UdpInterface::StartThread(void)
{
    mThread = std::thread(std::bind(&UdpInterface::Task, this));
}

/**
 * @brief Stop and wait the thread to quit.
 * @param None
 * @retval None
 */
void UdpInterface::StopThread(void)
{
    mQuit = true;
    mThread.join();
}

/**
 * @brief Send a message to RCU
 * @param msg Pointer to the message.
 * @param size Message size.
 * @retval False : failed to send; result : How many bytes were sent;
 */
int UdpInterface::SendMessageToVcu(uint8_t *msg, uint32_t size)
{
    if (mSocket <= 0)
    {
        ROS_ERROR("UdpInterface::SendMessageToVcu() error. mSocket <= 0 !!!");
        return false;
    }
    socklen_t sockLen = sizeof(mVcuAddr);
    int result = sendto(mSocket, msg, size, 0, (struct sockaddr *)&mVcuAddr, sockLen);
    if (result < 0)
    {
        ROS_ERROR("UdpInterface::Task() sendto() error. Error no : %d, error : %s", errno, strerror(errno));
        return false;
    }
    else
        return result;
}

void UdpInterface::SetIpAndPort(const char *vcuIp, uint16_t vcuUdpPort)
{
    memset(&mVcuAddr, 0, sizeof(mVcuAddr));
    mVcuAddr.sin_family = AF_INET;
    mVcuAddr.sin_addr.s_addr = inet_addr(vcuIp);
    mVcuAddr.sin_port = htons(vcuUdpPort);
}

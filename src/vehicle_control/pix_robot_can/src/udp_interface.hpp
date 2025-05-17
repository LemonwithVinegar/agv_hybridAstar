#pragma once

#include <stdint.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <thread>
#include <atomic>

#include "vcu_message.hpp"
#include "vcu_communicate.hpp"

//#define DEFAULT_VCU_IP "192.168.101.254"
#define DEFAULT_VCU_IP "10.0.0.254"
#define DEFAULT_VCU_IP_VCU_UDP_PORT 19000

class UdpInterface
{
public:
    void StartThread();
    void StopThread();
    int SendMessageToVcu(uint8_t *, uint32_t);
    static UdpInterface &GetInstance() { return instance; }
    void SetIpAndPort(const char *vcuIp = DEFAULT_VCU_IP, uint16_t vcuUdpPort = DEFAULT_VCU_IP_VCU_UDP_PORT);

private:
    UdpInterface();
    void operator=(const UdpInterface &) = delete;
    UdpInterface(const UdpInterface &) = delete;

    void Task();

private:
    static UdpInterface instance;

protected:
    static const uint32_t MAX_MSG_LEN_TO_SEND = 128;
    static const uint32_t RECV_MSG_BUF_LEN = 128;

private:
    int mSocket = 0;
    struct sockaddr_in mVcuAddr;
    std::thread mThread;
    std::atomic<bool> mQuit;
    uint8_t mRecvMsg[RECV_MSG_BUF_LEN];
};

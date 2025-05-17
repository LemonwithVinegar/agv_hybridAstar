#ifndef __UDP_SERVER_H_
#define __UDP_SERVER_H_

#include <iostream>
#include <string>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <sys/select.h>
#include <iomanip>

//来自客户端的原始信息
struct clientData{
    char recvieBuf[4096];
    struct sockaddr_in clientAddr;
};

//接收自UDP的遥控指令：包括前后左右与刹车指令
struct FakeRemoteCommand{
    uint16_t msg; //帧头 MSG+8(5008)
    uint16_t brake;  //刹车 0刹车 1解除刹车
    int longitudinal; //纵向的
    int lateral;    //横向的
};


//自定义udp服务器类
class myudpServer
{
    private:
        in_addr_t UDP_SERVER_IP = INADDR_ANY;
        // in_addr_t UDP_SERVER_IP = inet_addr("192.168.222.102");
        uint16_t UDP_SERVER_PORT = 6188;
    public:
        bool timeoutFlag = false; //接收阻塞超时标志，超时为 true
        struct clientData clientdata;
        int init(); //udp初始化
        struct clientData getDataFrom(int __fd); //返回接收的数据
        struct FakeRemoteCommand dataAnalysis(char* raw_data); //原始数据解析（解析为遥控指令）
        //调试输出函数
        void debugPrint();
        void debugPrint(struct FakeRemoteCommand cmd);
};

int myudpServer::init(){
    //第一步：socket
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0); //UDP 使用 SOCK_DGRAM 报式传输
    if(sockfd == -1){
        // perror("socket error");
        ROS_ERROR("socket error");
        exit(1);
    }
    //第二步：bind : 绑定服务器自己的地址
    struct sockaddr_in localAddr;
    localAddr.sin_family = AF_INET;
    // localAddr.sin_addr.s_addr = inet_addr("127.0.0.1");
    localAddr.sin_addr.s_addr = this->UDP_SERVER_IP;
    localAddr.sin_port = htons(this->UDP_SERVER_PORT);
    bind(sockfd, (struct sockaddr*)&localAddr, sizeof(localAddr));

    return sockfd;
}

struct clientData myudpServer::getDataFrom(int __fd){
    struct timeval timeout; //设置超时时间
    timeout.tv_sec = 0;
    timeout.tv_usec = 500*1000;

    fd_set readfd;
    FD_ZERO(&readfd); // 描述集初始化
    FD_SET(__fd, &readfd); // __fd塞入描述集
    int sret = select(__fd+1, &readfd, NULL, NULL, &timeout); //监听__fd文件描述符的读事件

    
    if(sret == 0 /*接收阻塞超时*/){
        this->timeoutFlag = true;
        std::cout << "chaoshi" << std::endl;
    }else{ /*未超时*/
        this->timeoutFlag = false;
        socklen_t clientAddr_len = sizeof(this->clientdata.clientAddr);
        memset(this->clientdata.recvieBuf, 0, sizeof(this->clientdata.recvieBuf)); //清空缓存，为下一次接收做准备
        int ret = recvfrom(__fd, this->clientdata.recvieBuf, sizeof(this->clientdata.recvieBuf), 0, (struct sockaddr *)&this->clientdata.clientAddr, &clientAddr_len);
        if(ret == -1){
            // perror("udp server recvfrom error");
            ROS_ERROR_STREAM_THROTTLE(1,"udp server recvfrom error");
        }
        // this->debugPrint();

    }
    
    return this->clientdata;
}

struct FakeRemoteCommand myudpServer::dataAnalysis(char* raw_data){
    struct FakeRemoteCommand cmd;
    memcpy(&cmd, raw_data, sizeof(cmd));
    return cmd;
}

void myudpServer::debugPrint(){
    // char str[100];
    // sprintf(str, "I recvied [%s] from udp client! <ip:%s , port:%d>", this->clientdata.recvieBuf, inet_ntoa(this->clientdata.clientAddr.sin_addr), ntohs(this->clientdata.clientAddr.sin_port));
    std::string str;
    // std::cout << "I recvied [" << this->clientdata.recvieBuf << "] from udp client! <ip:" << inet_ntoa(this->clientdata.clientAddr.sin_addr) << " , port:" << ntohs(this->clientdata.clientAddr.sin_port) << ">" << std::endl;
    std::cout << "I recvied from udp client! <ip:" << inet_ntoa(this->clientdata.clientAddr.sin_addr) << " , port:" << ntohs(this->clientdata.clientAddr.sin_port) << ">" << std::endl;
    for(int i = 0; i < 14; i++)
        std::cout << std::hex << (int)this->clientdata.recvieBuf[i] << " ";
    std::cout << std::endl;
    // std::cout << str << std::endl;
}

void myudpServer::debugPrint(struct FakeRemoteCommand cmd){
        std::cout << "---1111111111111" << std::endl;
        std::cout << "msg: " << cmd.msg << std::endl;
        std::cout << "brake: " << cmd.brake << std::endl;
        std::cout << "longitudinal: " << cmd.longitudinal << std::endl;
        std::cout << "lateral: " << cmd.lateral << std::endl;
}

#endif
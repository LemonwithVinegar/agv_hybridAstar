#include <ros/ros.h>
#include <iostream>
#include "custom_msgs/ControlMode.h"
// #include "custom_msgs/FakeMode.h"
#include <thread>
#include "mypthread.h"
#include "myudpServer.h"
#include "msg.h"

#define DEBUG 1

//全局变量
static bool globalTimeoutFlag = false; //全局udp数据接收超时标志
static custom_msgs::ControlMode controlModeMsg;
static FakeRemoteCommand cmd;

//接收udp数据的子线程处理函数
void* udpRecevierPthread(void* arg){
    myudpServer udpserver;
    int sockfd = udpserver.init(); //udp初始化
    
    char tmprecvieBuf[1024];
    struct sockaddr_in tmpclientAddr;
    socklen_t tmpclientAddr_len = sizeof(tmpclientAddr);
    while(1){
        // recvfrom(sockfd, tmprecvieBuf, sizeof(tmprecvieBuf), 0, (struct sockaddr *)&tmpclientAddr, &tmpclientAddr_len);
        // if(tmpclientAddr.sin_addr == inet_addr("192.168.222.3")){
            udpserver.clientdata = udpserver.getDataFrom(sockfd);
            cmd = udpserver.dataAnalysis(udpserver.clientdata.recvieBuf);
            globalTimeoutFlag = udpserver.timeoutFlag; //修改与主线程的共享变量 globalTimeoutFlag
        // }
        
        #if DEBUG
        // udpserver.debugPrint();
        // udpserver.debugPrint(cmd);
        #endif
    }
}

// //获取来自TCP接收到的控制模式指令
// bool getModeFromTcpCallback(custom_msgs::FakeMode::Request &Req, custom_msgs::FakeMode::Response &Res){
//     controlModeMsg.mode_type = Req.FakeMode;
//     Res.res = true;
//     return true;
// }


int main(int argc, char **argv){
    ros::init(argc, argv, "udp_telecontrol_node"); //初始化节点
    ros::NodeHandle nh;

    ros::Publisher  pub_ControlMode = nh.advertise<custom_msgs::ControlMode>("controlmode_msg",1); // 话题发布 : 遥控控制信息
    // ros::ServiceServer srv_FakeModeSwitch = nh.advertiseService("FakeModeSwitch",getModeFromTcpCallback); //服务 : 伪模式切换

    // 开启线程 : 
    mypthread udpthread;
    udpthread.create(udpRecevierPthread);

    ros::Rate loop_rate(200);
    while(ros::ok()){
        ros::spinOnce();
        controlModeMsg.mode_type = 0x20; //测试遥控用，请关闭
        controlModeMsg.enable = 0; //无需急停，始终为0
        if( globalTimeoutFlag == true){ // UDP 接收超时或帧头不对（干扰数据）
            controlModeMsg.brake_enable = 1;
            controlModeMsg.speed_require = 0;
            controlModeMsg.angle_require = 0;
            pub_ControlMode.publish(controlModeMsg);
        }else{ //UDP 接收未超时
            if(cmd.msg == 5008){ //UDP 接收未超时
                controlModeMsg.brake_enable = cmd.brake;
                controlModeMsg.speed_require = cmd.longitudinal;
                controlModeMsg.angle_require = cmd.lateral;
                pub_ControlMode.publish(controlModeMsg);
            }
        }

        
        loop_rate.sleep();
    }
    
   
    return 0;
}

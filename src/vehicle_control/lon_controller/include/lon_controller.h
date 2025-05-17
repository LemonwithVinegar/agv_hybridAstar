//*****************************************************************
// 作者 ：杨东
// 时间 ：2019.04.15
// 版权 ：重庆邮电大学自动化学院汽车电子工程中心智能车辆技术团队
// 说明 ：
//*****************************************************************
#ifndef LON_CONTROLLER_H
#define LON_CONTROLLER_H

#include "ros/ros.h"
#include <cstdint>
#include <geometry_msgs/Twist.h>
#include "vcu_message.hpp" //含UdpSystemStatus_t
//#include <pix_robot_can/Motorinfo.h> //含pix_robot_can::Motorinfo

//调试开关宏
#define LONCONTROLLER_DEBUG

//-->>全局变量
extern struct VehicleStat veh_stat;
extern struct Request request_value;

//-->>数据结构
enum LON_STATUS{
   FORWARD_ENABLE = 0x0, BACK_ENABLE = 0x1, STOP_ENABLE = 0x2, AEB_ENABLE = 0x3
};

struct Request {
    uint8_t request_type;            //请求类型
    float  run_speed;                //行车速度
    float  stop_distance;            //前方停止距离
    float  aeb_distance;             //前方AEB停止距离
};

//-->>三种运行状态处理
/**
 * @brief run_PID_solve
 * @return
 */
geometry_msgs::Twist run_solve();
geometry_msgs::Twist back_solve();

geometry_msgs::Twist stop_solve();

bool isManualMode();
void linearStart(double requestSpeed, geometry_msgs::Twist &comvel);

//-->>消息回调外部借口

/**
 * @brief set_requre
 * @param arg
 */
void set_requre(struct Request &arg);
void get_run_mode(UdpSystemStatus_t &systemStatus);
// void getMotorInfo(UdpMotorInfo_t &motorinfo);
#endif //LON_CONTROLLER_H

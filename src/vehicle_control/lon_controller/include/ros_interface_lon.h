//*****************************************************************
// 作者 ：杨东
// 时间 ：2019.04.15
// 版权 ：重庆邮电大学自动化学院汽车电子工程中心智能车辆技术团队
// 说明 ：
//*****************************************************************
#ifndef ROS_INTERFACE_LON_H
#define ROS_INTERFACE_LON_H

#include "lon_controller.h"
//规划目标消息
#include "custom_msgs/Request.h"
#include <pix_robot_can/SystemStatus.h> //含pix_robot_can::SystemStatus



//-->>消息回调
/**
 * @brief run_req_callback
 * @param msg
 */
void run_req_callback(const custom_msgs::Request::ConstPtr &msg);
void run_mode_callback(const pix_robot_can::SystemStatus::ConstPtr &msg);
// void motorSpeedCallback(const pix_robot_can::Motorinfo::ConstPtr &motorinfoMsg);
#endif //ROS_INTERFACE_LON_H

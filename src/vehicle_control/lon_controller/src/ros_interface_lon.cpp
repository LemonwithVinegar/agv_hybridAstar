#include "ros_interface_lon.h"
#include <iostream>

#include <pix_robot_can/SystemStatus.h> //含pix_robot_can::SystemStatus
#include "vcu_message.hpp" //含UdpSystemStatus_t

using namespace std;

/**
 * @brief run_req_callback
 * @param msg
 */
void run_req_callback(const custom_msgs::Request::ConstPtr &msg)
{
    struct Request temp;
    temp.request_type  = msg->reques_type;
    temp.run_speed     = msg->run_speed;
    temp.aeb_distance  = msg->aeb_distance;
    temp.stop_distance = msg->stop_distance;
    set_requre(temp);
    return;
}

void run_mode_callback(const pix_robot_can::SystemStatus::ConstPtr &msg)
{
    UdpSystemStatus_t systemStatus;
    systemStatus.manualMode = msg->manualMode;
    while(ros::ok()){

    ROS_INFO_THROTTLE(1,"RUN: status = %d", systemStatus.manualMode);
    }
    //std::cout << "pix/systemStatus callback" << std::endl;
    // if(systemStatus.manualMode){
    //         std::cout << "manual" << std::endl;
    // }else
    //std::cout << "auto" << std::endl;
    get_run_mode(systemStatus);
    return;
}

// void motorSpeedCallback(const pix_robot_can::Motorinfo::ConstPtr &motorinfoMsg){
//     UdpMotorInfo_t motorinfo;
//     motorinfo.speed = motorinfoMsg.speed;
//     getMotorInfo(motorinfo);
// }


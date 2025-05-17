#include "lon_controller.h"
#include "vcu_message.hpp" //含UdpSystemStatus_t
#include<iostream>
//-->>全局变量
#define LONCONTROLLER_DEBUG
struct Request     request_value = {LON_STATUS::FORWARD_ENABLE,0,0,0};
UdpSystemStatus_t systemStatus_value;
//UdpMotorInfo_t motorInfo_value;

int manualModeFlag = 0;

void set_requre(struct Request &arg){
    request_value = arg;
    return ;
}

void get_run_mode(UdpSystemStatus_t &systemStatus){
  systemStatus_value.manualMode= systemStatus.manualMode;
  return;
}

// int* getMotorInfo(UdpMotorInfo_t &motorinfo){
//   //motorInfo_value.speed = motorinfo.speed;
//   return motorinfo.speed;
// }



bool isManualMode(){
  return systemStatus_value.manualMode;
}

// bool isMotorStop(){
//   return true;
// }


void linearStart(double requestSpeed, geometry_msgs::Twist &comvel){
  float unit = requestSpeed / 400.0;  //将目标速度分为200份逐渐加速
  if(isManualMode() == true){  //here should be true, now have a bug so be false
    //std::cout<<"dddddddd"<<std::endl;
    manualModeFlag = 0;
    comvel.linear.x = manualModeFlag * unit;
  }else{
    comvel.linear.x = manualModeFlag * unit;
    manualModeFlag++;
    if( comvel.linear.x>0.65){
       comvel.linear.x =0.65;
       manualModeFlag=200;
    }else if( comvel.linear.x<-0.2){
      comvel.linear.x = -0.2;
       manualModeFlag=200;
    }
    
  }
}

/**
 * @brief run_solve
 * @return
 */
geometry_msgs::Twist run_solve() {
  geometry_msgs::Twist temp;
  temp.linear.x = request_value.run_speed;
 // temp.trq_value  = request_value.run_speed;  //设置速度
  linearStart(request_value.run_speed, temp); //线性起步
  #ifdef LONCONTROLLER_DEBUG
  ROS_INFO_THROTTLE(1,"RUN: Speed = %f", temp.linear.x);
  #endif
  
  return temp;
}

geometry_msgs::Twist back_solve() {
  geometry_msgs::Twist temp;
  temp.linear.x = request_value.run_speed;
 // temp.trq_value  = request_value.run_speed;  //设置速度
  linearStart(request_value.run_speed, temp); //线性起步
  #ifdef LONCONTROLLER_DEBUG
  ROS_INFO_THROTTLE(1,"RUN: Speed = %f", temp.linear.x);
  #endif
  
  return temp;
}

geometry_msgs::Twist stop_solve() {
  geometry_msgs::Twist temp;
  temp.linear.x = 0;
 // temp.trq_value  = request_value.run_speed;  //设置速度
  // linearStart(-0.4, temp); //线性起步
  #ifdef LONCONTROLLER_DEBUG
  ROS_INFO_THROTTLE(1,"RUN: Speed = %f", temp.linear.x);
  #endif
  
  return temp;
}

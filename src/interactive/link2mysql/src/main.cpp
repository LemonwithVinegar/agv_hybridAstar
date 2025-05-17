
#include "iostream"
#include "string"
#include "udp_core.h"

#include <ros/ros.h>
#include <ros/time.h>
#include <custom_msgs/NaviData.h>
#include <sensor_msgs/Imu.h>
#include <custom_msgs/VehicleStat.h>
#include <tf/transform_datatypes.h>

custom_msgs::NaviData   navi_data;
custom_msgs::VehicleStat vehic_stat;

void onNaviMsgRecvd(const custom_msgs::NaviData::ConstPtr &msg){
   navi_data = *msg;
}

void onVehicleStatRecvd(const custom_msgs::VehicleStat::ConstPtr &msg){
   vehic_stat = *msg;
}

int main(int argc, char **argv){

   ros::init(argc, argv, "link2mysql");
   ros::NodeHandle nh;

   ros::Subscriber sub_navi = nh.subscribe("/navi_msg",1,onNaviMsgRecvd);
   ros::Subscriber sub_vehic = nh.subscribe("/VehicleStat",1,onVehicleStatRecvd);

   udp_core core;
   core.socket_init(6623,"127.0.0.1",12345,2.0);

   std::string sendBuff;
   ros::Rate rate(10);
   while (ros::ok()){
      ros::spinOnce();
      sendBuff.clear();
      sendBuff += "1 ";
      sendBuff += std::to_string(navi_data.speed2d);
      sendBuff += " ";
      sendBuff += std::to_string(navi_data.longitude);
      sendBuff += " ";
      sendBuff += std::to_string(navi_data.latitude);
      sendBuff += " ";
      sendBuff += std::to_string(0);
      sendBuff += " ";
      sendBuff += std::to_string(vehic_stat.RemainingMile);
      sendBuff += " ";
      sendBuff += std::to_string(vehic_stat.TolMileage);
      sendBuff += " ";
      sendBuff += std::to_string(navi_data.heading);
      sendBuff += " ";
      sendBuff += std::to_string(0);
      sendBuff += " ";
      sendBuff += std::to_string(0);
      sendBuff += " ";
      sendBuff += std::to_string(0);
      sendBuff += " ";
      sendBuff += std::to_string(0);
      sendBuff += " ";
      sendBuff += std::to_string(0);
      sendBuff += "\n";

      //std::cout << sendBuff;

      core.send_data(sendBuff.c_str(),sendBuff.size());

      rate.sleep();
   }
}
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>

#include <ros/ros.h>
#include <ros/time.h>

#include "custom_msgs/TrafficLight.h"
#include "custom_msgs/GateStat.h"

#include "udp_core.h"


using namespace std;

int light;  //0 红 1 黄 2 绿
int l_time;

bool gata_stat = false;



bool onservCallRecvd(custom_msgs::TrafficLight::Request &req,custom_msgs::TrafficLight::Response &res){
    switch (light)
    {
    case 0:
        res.color = custom_msgs::TrafficLight::Response::RED;
        break;
    case 1:
        res.color = custom_msgs::TrafficLight::Response::YELLOW;
        break;
    case 2:
        res.color = custom_msgs::TrafficLight::Response::GREEN;
        break;
    default:
        break;
    }
    res.time = l_time;
}

bool onGateServCallRecvd(custom_msgs::GateStat::Request &req,custom_msgs::GateStat::Response &res){
    res.stat = gata_stat;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "interactive_node");
    ros::NodeHandle nh;

    ros::ServiceServer traffic_light_service = nh.advertiseService("/traffic_light_service", onservCallRecvd);
    ros::ServiceServer gate_service = nh.advertiseService("/gate_service", onGateServCallRecvd);

    light  = 0 ;
    l_time = 20;

    udp_core core;
    core.socket_init(6677,"127.0.0.1",6688,2.0);
    char rec_buff[8];

    ros::Timer timer1 = nh.createTimer(ros::Duration(1),
                                      [](const ros::TimerEvent& evnt){
                                        l_time--;
                                        light  = (l_time <= 0) ? ++light : light;
                                        l_time = (l_time <= 0) ? 20 : l_time;
                                        light  = (light  >= 3) ? 0  : light;
                                        //cout << "light : " << light << "  time : "<<l_time <<endl;
                                    });
    ros::Timer timer2 = nh.createTimer(ros::Duration(0.2),
                                      [&rec_buff,&core](const ros::TimerEvent& evnt){
                                        //memset(rec_buff,0,8);
                                        core.recv_data(rec_buff,8);
                                        if(strncmp(rec_buff,"1",1)){
                                            gata_stat = 0;
                                        }else{
                                            gata_stat = 1;
                                        }
                                        //std::cout << gata_stat << std::endl;
                                    });

    while(ros::ok()){

        // core.recv_data(rec_buff,8);

        // std::cout << rec_buff << std::endl;

        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }

    return 0;
}


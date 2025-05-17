#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>

#include <ros/ros.h>
#include <ros/time.h>
#include "custom_msgs/App.h"
#include "custom_msgs/GateStat.h"
#include "udp_core.h"

using namespace std;



bool link2app(int enable,std::string &buff,ros::ServiceClient link2app_client){
    custom_msgs::App srv;
    srv.request.isSend = enable;
    srv.request.sendbuf = buff;
    if (link2app_client.call(srv)){
        if(enable==0){
            buff = srv.response.recvbuf;
            cout << buff << endl;
        }
        return 1;
    }
    else{
        ROS_ERROR("test : Failed to call service app");
        return 0;
    }
}

bool Get_gate_stat(int id,ros::ServiceClient gate_client){
    custom_msgs::GateStat srv;
    srv.request.id = id;
    if (gate_client.call(srv)){
        return srv.response.stat;
    }
    else{
        ROS_ERROR("test : Failed to call service gate");
        return 0;
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "linktest_node");
    ros::NodeHandle nh;
    ros::ServiceClient gate_client  = nh.serviceClient<custom_msgs::GateStat>("/gate_service");
    ros::ServiceClient link2app_client  = nh.serviceClient<custom_msgs::App>("/link2app_service");



    string buff = "testing\n";

 

    while(ros::ok()){
        link2app(1,buff,link2app_client);

        cout << Get_gate_stat(1,gate_client) << endl;

        ros::spinOnce();
        ros::Duration(1).sleep();
    }

    return 0;
}


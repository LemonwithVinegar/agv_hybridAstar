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

udp_core udp_core;
char recvdata[50];
int ret=0;


bool feil_gata_stat = false;
bool xusuo_gata_stat = false;

string last_str;

bool onLinkCallRecvd(custom_msgs::App::Request &req,custom_msgs::App::Response &res){
    if(req.isSend == 1){ 
        ret = udp_core.send_data(req.sendbuf.c_str(),req.sendbuf.size());
        ROS_WARN_STREAM_THROTTLE(2,"send2APP : "<<req.sendbuf.c_str());
        if(ret == -1){
            cout<<"link2app: send failed !"<<endl;
        }
        return true;  
    }else{ 
        bzero(recvdata,50);
        ret = udp_core.recv_data(recvdata,50);
        if(ret == -1){
            cout<<"link2app:recv failed !"<<endl;
            //res.recvbuf = "NULL";
            //return true;
        }
        //cout << ret << endl;
        if(ret>0){
            ROS_WARN_STREAM_THROTTLE(2,"recvfAPP : "<<recvdata);
            res.recvbuf = recvdata;
            last_str = recvdata;
        }else{
            res.recvbuf = last_str;
        }

        // if(res.recvbuf == "DoorOpened")
        //     feil_gata_stat = false;
        // else
        //     feil_gata_stat = true;
         

        // if(res.recvbuf == "DoorOpenedXushuo")
        //     xusuo_gata_stat = false;
        // else
        //     xusuo_gata_stat = true;

        return true; 
    }
}

bool onGateServCallRecvd(custom_msgs::GateStat::Request &req,custom_msgs::GateStat::Response &res){
    switch (req.id){
        case 0:
            res.stat = xusuo_gata_stat;
            break;
        case 1:
            res.stat = feil_gata_stat;
            break;
        
        default:
            break;
    }
    return true;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "link2app_node");
    ros::NodeHandle nh;

    ros::ServiceServer link2app_service = nh.advertiseService("/link2app_service", onLinkCallRecvd);
    ros::ServiceServer gate_service = nh.advertiseService("/gate_service", onGateServCallRecvd);

    udp_core.socket_init(10001,"192.168.1.110",12580,2.0);
    // udp_core.socket_init(6677,"127.0.0.1",6688,2.0);

    ros::spin();

    // while(ros::ok()){

    //     ros::spinOnce();
    //     ros::Duration(0.1).sleep();
    // }

    return 0;
}


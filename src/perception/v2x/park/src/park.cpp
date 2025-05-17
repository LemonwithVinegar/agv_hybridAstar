#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <vector>
#include <algorithm>

#include <ros/ros.h>
#include <ros/time.h>
#include <custom_msgs/CurPose.h>
// #include <custom_msgs/ParkSpace.h>
#include "custom_msgs/TrafficLight.h"

#include "udp_core.h"


using namespace std;

#pragma pack(1)
typedef struct PARK_SPACE{
    bool   valid;                      // 0:invalid 1:valid
    double x;                         //point_x 
    double y;                         //point_y

    int traffic_1_color;              //color: 0 invalid 1 green left 2 green straight 3 red left 4 red straight
    float traffic_1_x;
    int traffic_2_color;              //color: 0 invalid 1 green left 2 green straight 3 red left 4 red straight
    float traffic_2_x;
    int traffic_3_color;              //color: 0 invalid 1 green left 2 green straight 3 red left 4 red straight
    float traffic_3_x;

} tPARK_SPACE;

#pragma pack()

int cur_pose_s;
tPARK_SPACE park_space;

bool llight = 0;
bool dlight = 0;
 
std::map<int,int> recv_color;
         
void onCurPoseSDRecvd(const custom_msgs::CurPose::ConstPtr &msg){
    cur_pose_s = (*msg).s;
}

// int Data_analysis(int *color, int phase_id)
// {
//     //0 light
//     if(park_space.traffic_1_color == 0) return 0;
//     //1 light
//     if(park_space.traffic_2_color == 0){
//         if(park_space.traffic_1_color < 3) *color = 1;
//         else *color = 2;
//         return 1;
//     }
//     //2 light
//     if(park_space.traffic_3_color == 0){
//         if(phase_id == 0){
//             int res_color;
//             if(park_space.traffic_1_x<park_space.traffic_2_x){
//                 res_color = park_space.traffic_1_color;
//             }
//             else{
//                 res_color = park_space.traffic_2_color;
//             }
//             if(res_color < 3) *color = 1;
//             else *color = 2;
//         }
//         if(phase_id == 1){
//             int res_color;
//             if(park_space.traffic_1_x<park_space.traffic_2_x){
//                 res_color = park_space.traffic_2_color;
//             }
//             else{
//                 res_color = park_space.traffic_1_color;
//             }
//             if(res_color < 3) *color = 1;
//             else *color = 2;
//         }
//         return 1;
//     }
//     //3 light
    
//     vector<float> x;
//     x.push_back(park_space.traffic_1_x);
//     x.push_back(park_space.traffic_2_x);
//     x.push_back(park_space.traffic_3_x);
//     sort(x.begin(),x.end());
//     if(phase_id == 0){
//         int res_color;
//         if(x[0] == park_space.traffic_1_x){
//             res_color = park_space.traffic_1_color;
//         }
//         else if(x[0] == park_space.traffic_2_x){
//             res_color = park_space.traffic_2_color;
//         }
//         else{
//             res_color = park_space.traffic_3_color;
//         }
//         if(res_color < 3) *color = 1;
//         else *color = 2;
//     }
//     if(phase_id == 1){
//         int res_color;
//         if(x[1] == park_space.traffic_1_x){
//             res_color = park_space.traffic_1_color;
//         }
//         else if(x[1] == park_space.traffic_2_x){
//             res_color = park_space.traffic_2_color;
//         }
//         else{
//             res_color = park_space.traffic_3_color;
//         }
//         if(res_color < 3) *color = 1;
//         else *color = 2;
//     }
//     if(phase_id == 2){
//         int res_color;
//         if(x[2] == park_space.traffic_1_x){
//             res_color = park_space.traffic_1_color;
//         }
//         else if(x[2] == park_space.traffic_2_x){
//             res_color = park_space.traffic_2_color;
//         }
//         else{
//             res_color = park_space.traffic_3_color;
//         }
//         if(res_color < 3) *color = 1;
//         else *color = 2;
//     }

//     return 1;
// }

bool onTrafficLightCallRecvd(custom_msgs::TrafficLight::Request  &req,
                             custom_msgs::TrafficLight::Response &res)
{

    //    请在此处为消息响应结构体赋值
    //    int32 color             # 1 绿色 2 红色 3 黄色 -1无效
    //    int32 time              # 剩余时间
    //    请为 res.color
    //        res.time 两个变量赋值

    //ROS_INFO("Traffic Light request received");
    //int color=0;
    //int phase_id = req.phase_id;

    if(llight){
        res.color = custom_msgs::TrafficLight::Response::GREEN;
    }else{
        res.color = custom_msgs::TrafficLight::Response::RED;
    }

    // if(req.phase_id == 0){
    //     if(llight){
    //         res.color = custom_msgs::TrafficLight::Response::GREEN;
    //     }else{
    //         res.color = custom_msgs::TrafficLight::Response::RED;
    //     }
    // }

    // if(req.phase_id == 1){
    //     if(dlight){
    //         res.color = custom_msgs::TrafficLight::Response::GREEN;
    //     }else{
    //         res.color = custom_msgs::TrafficLight::Response::RED;
    //     }
    // }

    //Data_analysis(&color,phase_id);

    //res.color = (int)color;
    //res.time = (int)time;
    //ROS_INFO("Traffic Light is %d",(int)color);
    return true;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "park_node");
    ros::NodeHandle nh;
                recv_color[0] = 0;

            recv_color[1] = 0;
            recv_color[2] = 0;
            recv_color[3] = 0;
            recv_color[4] = 0;
    udp_core udp_core;

    // ros::Publisher  pub_parkSpace;
    ros::Subscriber sub_cur_pose;

    // custom_msgs::ParkSpace ros_parkSpace;

    sub_cur_pose     = nh.subscribe("/cur_pose_all", 1 , onCurPoseSDRecvd);
    // pub_parkSpace    = nh.advertise<custom_msgs::ParkSpace>("/park_space",1,true);//发布
    ros::ServiceServer service = nh.advertiseService("traffic_light_service", onTrafficLightCallRecvd);

    udp_core.socket_init(3344,"192.168.1.105",5566,0.1);
   
    //ros::spin();
    int count = 0;
    while(ros::ok()){
        // if(cur_pose_s > 2961){
        //     udp_core.send_data("1",1);

        //     //bzero((void*)park_space,sizeof(park_space));           
        // }
        bzero(&park_space,sizeof(park_space));
        udp_core.recv_data((char*)&park_space,sizeof(park_space));

        // ros_parkSpace.valid = park_space.valid;
        // ros_parkSpace.x = park_space.x;
        // ros_parkSpace.y = park_space.y;

        // ROS_INFO_STREAM_THROTTLE(1,"park_space.x  : "<< park_space.x);
        // ROS_INFO_STREAM_THROTTLE(1,"park_space.y  : "<< park_space.y);
        // ROS_INFO_STREAM_THROTTLE(1,"park_space.valid  : "<< park_space.valid);



        //pub_parkSpace.publish(ros_parkSpace);
        //std::cout<<park_space.x<<std::endl;

        ROS_INFO_STREAM_THROTTLE(1,"park_space.traffic_1_color: "<< park_space.traffic_1_color);
        ROS_INFO_STREAM_THROTTLE(1,"park_space.traffic_2_color: "<< park_space.traffic_2_color);
    
        recv_color[park_space.traffic_1_color]++;
        recv_color[park_space.traffic_2_color]++;
        count++;
        if(count == 6){
            if(recv_color[1]> recv_color[3]){
                llight = 1;
            
            }else{
                llight = 0;
            }

            if(recv_color[2]>recv_color[4]){
                dlight = 1;
            }else{
                dlight = 0;
            }

            count=0;
            recv_color[0] = 0;
            recv_color[1] = 0;
            recv_color[2] = 0;
            recv_color[3] = 0;
            recv_color[4] = 0;
        }


        ros::spinOnce();
        ros::Duration(0.04).sleep();
    }

    return 0;
}




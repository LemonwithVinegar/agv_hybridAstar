#include "ros/ros.h"
#include <custom_msgs/Ambulance.h>
#include <custom_msgs/TrafficLight.h>
#include <boost/thread.hpp> 


int color = 1; //绿灯2，红灯1，黄灯0
int count = 30;
int last_color = 2;


bool onAmbulanceCallRecvd(custom_msgs::Ambulance::Request &req,
                          custom_msgs::Ambulance::Response &res)
{
    res.IsAmbulance = true;
    
    return true;
}

bool onTrafficLightCallRecvd(custom_msgs::TrafficLight::Request &req,
custom_msgs::TrafficLight::Response &res){
//    res.color = color;
//    res.time = count;
    //ROS_INFO("----------------------1111111111");
    if(color == 1){
        res.color = custom_msgs::TrafficLight::Response::RED;
    }
    if(color == 2){
        res.color = custom_msgs::TrafficLight::Response::GREEN;
    }
    if(color == 0){
        res.color = custom_msgs::TrafficLight::Response::YELLOW;
    }
    
    res.time = count;
    ROS_INFO_STREAM_THROTTLE(1,"fake : color: "<< res.color<<" time: "<< res.time);
    
    return true;
}

void fakeTrafficLightRun(){
    ros::Rate loop_rate(1);
    while(true){
        if(--count == 0){
            switch (color){
                case 1:   //绿
                    color = 3;
                    count = 3;
                    last_color = 1;
                    break;
                case 3:
                    if(last_color == 2){
                        color = 1;
                        count = 30;
                    }
                    else{
                        color = 2;
                        count = 30;
                    }
                    break;
                case 2:
                    color = 3;
                    count = 3;
                    last_color = 2;
                    break;
            }
        }
        loop_rate.sleep();
        ROS_INFO("--------------color: %d, time: %d",color,count);
    }
}

/**
* @brief 主函数
*
*/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "obu_node");
    ros::NodeHandle nh;
    ros::ServiceServer avoid_ambulance_service = nh.advertiseService("avoid_amulance_service", onAmbulanceCallRecvd);
    ros::ServiceServer traffic_light_service = nh.advertiseService("traffic_light_service", onTrafficLightCallRecvd);
    //以上为ros相关的初始化逻辑
    boost::thread thrd(fakeTrafficLightRun);
    
    while (ros::ok()) {
        ros::spinOnce(); //轮询一次，是否有交通灯的查询请求，如果有就进入onTrafficLightCallRecvd回调函数
        usleep(100*1000); //休眠100ms后再查询是否有查询请求
    }

    return 0;
}

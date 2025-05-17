#include "ros/ros.h"
#include "custom_msgs/TrafficLight.h"
#include <custom_msgs/Ambulance.h>
#include <custom_msgs/PickUp.h>


extern "C"{
#include "app.h"
#include "V2X_Thread.h"
}

pthread_mutex_t mybuffer_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_t V2X_ThreadID;

/**
* @brief Service回调函数,当接收到其他ros节点发出的交通灯查询请求时才执行
* @param[in] req    请求内容
* @param[out] res   响应内容
*/
bool onTrafficLightCallRecvd(custom_msgs::TrafficLight::Request  &req,
                             custom_msgs::TrafficLight::Response &res)
{
    //    查询交通灯的请求结构体req中包含一下三个信息
    //    float32 direction       #车辆即将开往的方向，单位°，以车头为0，顺时针为正方向
    //    float64 longitude       #交通灯所在路口中心的经度
    //    float64 latitude        #交通灯所在路口中心的纬度
    //    分别从 req.direction
    //          req.longitude
    //          req.latitude 三个变量中得到


    //    ...
    //    ...
    //    ...



    //    请在此处为消息响应结构体赋值
    //    int32 color             # 1 绿色 2 红色 3 黄色 -1无效
    //    int32 time              # 剩余时间
    //    请为 res.color
    //        res.time 两个变量赋值

    // double color=0.0, time=0.0;
    // int IsPickUp = 0, IsAmbulance = 0;
    // Data_analysis(req.direction, &color, &time, &IsPickUp, &IsAmbulance);

    res.color = msg.status;//color;
    res.time = msg.lefttime;//time;
    // res.color = 3;//color;
    // res.time = 20;//time;

    return true;
}
/**
* @brief 主函数
*
*/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "obu_node");
    ros::NodeHandle nh;
    ros::ServiceServer service = nh.advertiseService("traffic_light_service", onTrafficLightCallRecvd);
    //以上为ros相关的初始化逻辑

    int ret = -ENOSYS;
    ret = pthread_create(&V2X_ThreadID, NULL, Obu_Thread, NULL);
    if (ret)
    {
        errno = ret;
        perror("Obu_Thread fail");
        exit(EXIT_FAILURE);
    }
    else
    {
        printf("create Obu Thread success\n");
    }

    while (ros::ok()) {
        ros::spinOnce(); //轮询一次，是否有交通灯的查询请求，如果有就进入onTrafficLightCallRecvd回调函数
        usleep(100*1000); //休眠100ms后再查询是否有查询请求
    }

    //由于这个while循环会阻塞住整个进程，所以，UDP链接的监听要么另外开一个线程，要么去掉该while循环，在UDP的监听循环中适时地调用ros::spinOnce();
    //pthread_mutex_destroy(&mybuffer_mutex);
    return 0;
}

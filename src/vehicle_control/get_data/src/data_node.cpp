#include <vector>
#include <cmath>
#include "Eigen/Core"
#include <geometry_msgs/Twist.h>
#include <iostream>
#include "ros/ros.h"

#include "custom_msgs/SteeringCmd.h"
#include "custom_msgs/VehicleStat.h"
#include "custom_msgs/Path.h"
#include "geometry_msgs/Pose2D.h"
#include "custom_msgs/TurnLightCmd.h"
#include "custom_msgs/Request.h"

struct VehicleStat {
    double VehicleSpeed; //车速
    double angle;//前轮转角
    double x;
    double y;
    double theta;
};
struct VehicleStat vehicleStat; // 注意这里修改为小写的vehicleStat
// AGV没有相应的反馈，获取的速度、前轮转角是通过控制命令下发的
void MySpeedCallbacks(const geometry_msgs::Twist::ConstPtr &msg) {
    vehicleStat.VehicleSpeed = msg->linear.x;//车速

    ROS_INFO("speed = %f", vehicleStat.VehicleSpeed);
}

void MyAngleCallbacks(const geometry_msgs::Twist::ConstPtr &msg) {
    vehicleStat.angle = msg->angular.z;//前轮转角

    ROS_INFO("pix/com_vel recived");
    ROS_INFO("geometry_msgs::Twist msg->angular.z = %f", vehicleStat.angle);
}

void onCurPoseMsgRecvd(const geometry_msgs::Pose2D::ConstPtr &msg) {
    vehicleStat.theta = msg->theta;//航向角
    vehicleStat.x = msg->x;
    vehicleStat.y = msg->y;

    // ROS_INFO_STREAM_THROTTLE(1,"x = : "<< aft_pose2d.x);
    std::cout << "VehicleStat.y: " << vehicleStat.y << std::endl;
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "data_node");
    ros::NodeHandle n_handle;

    ros::Subscriber subLonGeometryTwist = n_handle.subscribe("pix/lon_com_vel", 1, MySpeedCallbacks); //速度
    ros::Subscriber subLatGeometryTwist = n_handle.subscribe("pix/lat_com_vel", 1, MyAngleCallbacks); //转角
    ros::Subscriber cur_pose_sub_ = n_handle.subscribe("/cur_pose", 1, onCurPoseMsgRecvd);            //x,y,theta

    // 打开文件进行写入，如果文件不存在就新建
    FILE *file = fopen("/home/nvidia/Desktop/yc/agv_hybridAstar/src/vehicle_control/get_data/data/vehicle_data_log.txt", "a");


    if (file == NULL) {
        ROS_ERROR("无法打开文件");
        return 1;
    }
    //-->>设置主线程循环周期
    ros::Rate loop_rate(20);//50ms
    double t=0;
    while (ros::ok()) {
        // 在这里使用fprintf将数据写入文件
        fprintf(file, "%.2f %.2f %.2f %.2f%.2f %.2f\n", t,vehicleStat.VehicleSpeed, vehicleStat.angle, vehicleStat.x, vehicleStat.y, vehicleStat.theta);         
        t += 0.05;
        ros::spinOnce();
        loop_rate.sleep();
    }
    // 关闭文件
    fclose(file);
    return 0;
}

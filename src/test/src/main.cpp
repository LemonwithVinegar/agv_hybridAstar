#include <ros/ros.h>

#include <custom_msgs/SlamPose.h>
#include <custom_msgs/UWBPose.h>
#include <geometry_msgs/PoseStamped.h>
#include <custom_msgs/SLAMReboot.h>
#include <unistd.h>

#include <iostream>
using namespace std;


int main(int argc, char **argv){
    ros::init(argc, argv, "leastsquare");
    ros::NodeHandle nh;

    ros::ServiceClient locationRebootClient = nh.serviceClient<custom_msgs::SLAMReboot>("location_reboot");
    ros::ServiceClient slamRebootClient = nh.serviceClient<custom_msgs::SLAMReboot>("slam_reboot");
    ros::ServiceClient gridRebootClient = nh.serviceClient<custom_msgs::SLAMReboot>("grid_reboot");
    custom_msgs::SLAMReboot srv;
    cout << "come in-----" <<endl;
    if(slamRebootClient.call(srv)) {
        cout << "Call slamRebootService Success!" << endl;
        // ROS_INFO("Call Service Fail!");
    }

    if(gridRebootClient.call(srv)) {
        cout<<"Call gridRebootService Success!"<<endl;
        // ROS_INFO("Call gridRebootService Success!");
    }
    usleep(1000*500);
    //mainRebootClient.call(srv);
    if(locationRebootClient.call(srv)) {
        cout<<"Call locationRebootService Success!"<<endl;
        // ROS_INFO("Call locationRebootService Success!");
    }
    
    cout << "over" << endl;
    return 0;
}

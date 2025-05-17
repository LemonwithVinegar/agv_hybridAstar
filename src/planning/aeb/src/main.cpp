#include "ros/ros.h"
#include "autoemergencybraking.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "aeb_node");
    AutoEmergencyBraking aeb_node;
    aeb_node.exec(argc,argv);
    return 0;
}

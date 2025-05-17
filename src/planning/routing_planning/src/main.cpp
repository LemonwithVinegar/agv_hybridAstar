#include "ros/ros.h"
#include "node_routing_core.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "routing_planning_node");
    Node_Routing_Core node_routing;
    node_routing.exec();

    return 0;
}

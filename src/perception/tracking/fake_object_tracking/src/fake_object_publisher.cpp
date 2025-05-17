#include "ros/ros.h"
#include "std_msgs/String.h"
#include <custom_msgs/ObjectArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fake_object_publisher");
    ros::NodeHandle nh;

    ros::Publisher chatter_pub = nh.advertise<custom_msgs::ObjectArray>("ObjectArray", 1);
    ros::Publisher pub_vis = nh.advertise<visualization_msgs::Marker>("objectvis", 1);
    custom_msgs::ObjectArray obj_array;
    custom_msgs::Object fake_object;
    fake_object.id = 1;
    fake_object.vx = 0;
    fake_object.vy = 0;
    fake_object.type = 1;
//    fake_object.x_pos = -17.102;
//    fake_object.y_pos = 4.7555;  //苏大佬假设的
    fake_object.x_pos = 120.211;
    fake_object.y_pos = 4.8443;

    obj_array.objs.push_back(fake_object);

    fake_object.id = 1;
    fake_object.vx = 0;
    fake_object.vy = 0;
    fake_object.type = 1;
//    fake_object.x_pos = -17.102;
//    fake_object.y_pos = 4.7555;  //苏大佬假设的
//    fake_object.x_pos = 122.170;
//    fake_object.y_pos = -0.072;
    fake_object.x_pos = -13.6117;
    fake_object.y_pos = 5.501;
    
    obj_array.objs.push_back(fake_object);

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.ns = "obstacle";
    marker.id = 1;
    marker.lifetime = ros::Duration(0.1);
    marker.scale.x = 0.1;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.a =1;
    marker.color.r = 0;
    marker.color.g = 1;
    marker.color.b = 0;
    marker.type = visualization_msgs::Marker::LINE_LIST;

    geometry_msgs::Point p;
    p.x=-17.102;
    p.y=4.7555;
    p.z=-1.7;
    
    geometry_msgs::Point points[8];
    points[0].x = 120.211-0.5;
    points[0].y = 4.8443;
    points[0].z = 0;

    points[1].x = 120.211+0.5;
    points[1].y = 4.8443;
    points[1].z = 0;

    points[2].x = 120.211+0.5;
    points[2].y = 4.8443-0.5;
    points[2].z = 0;

    points[3].x = 120.211-0.5;
    points[3].y = 4.8443-0.5;
    points[3].z = 0;

    points[4].x = -13.6117-0.5;
    points[4].y = 5.501;
    points[4].z = 0;

    points[5].x = -13.6117+0.5;
    points[5].y = 5.501;
    points[5].z = 0;

    points[6].x = -13.6117+0.5;
    points[6].y = 5.501-0.5;
    points[6].z = 0;

    points[7].x = -13.6117-0.5;
    points[7].y = 5.501-0.5;
    points[7].z = 0;

    marker.points.push_back(points[0]);
    marker.points.push_back(points[1]);
    marker.points.push_back(points[1]);
    marker.points.push_back(points[2]);
    marker.points.push_back(points[2]);
    marker.points.push_back(points[3]);
    marker.points.push_back(points[3]);
    marker.points.push_back(points[0]);

    marker.points.push_back(points[4]);
    marker.points.push_back(points[5]);
    marker.points.push_back(points[5]);
    marker.points.push_back(points[6]);
    marker.points.push_back(points[6]);
    marker.points.push_back(points[7]);
    marker.points.push_back(points[7]);
    marker.points.push_back(points[4]);
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        chatter_pub.publish(obj_array);
        pub_vis.publish(marker);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

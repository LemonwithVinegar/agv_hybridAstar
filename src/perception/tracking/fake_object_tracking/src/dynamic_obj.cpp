#include "ros/ros.h"
#include "std_msgs/String.h"
#include <custom_msgs/ObjectArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

class FakeObject{
public:
    static int count;
    FakeObject(float x,float y,float vx,float vy);
    visualization_msgs::Marker marker;
    custom_msgs::Object obj;

public:
    void reckon(float vx=0,float vy=0);
    void calculateBox(float x,float y,float w,float l);

private:
    std::vector<geometry_msgs::Point> points;
};

int FakeObject::count =1;

FakeObject::FakeObject(float x, float y, float vx, float vy)
{
    points.resize(5);

    obj.id = count++;
    obj.vx = vx;
    obj.vy = vy;
    obj.type = 1;
    obj.x_pos = x;
    obj.y_pos = y;
    obj.lwh.x = 5;
    obj.lwh.y = 2;

    marker.header.frame_id = "map";
    marker.ns = "obstacle";
    marker.id = obj.id;
    marker.lifetime = ros::Duration(0.1);
    marker.scale.x = 0.1;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.a =1;
    marker.color.r = 0;
    marker.color.g = 1;
    marker.color.b = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
}

void FakeObject::reckon(float vx, float vy)
{
    obj.x_pos += vx*0.1;
    obj.y_pos += vy*0.1;
    obj.vx = vx;
    obj.vy = vy;

    calculateBox(obj.x_pos,
                 obj.y_pos,
                 obj.lwh.y,
                 obj.lwh.x);
}

void FakeObject::calculateBox(float x, float y, float w, float l)
{
    w=w/2;
    l=l/2;

    points[0].x = x-w;
    points[0].y = y-l;
    points[0].z = 0;

    points[1].x = x+w;
    points[1].y = y-l;
    points[1].z = 0;

    points[2].x = x+w;
    points[2].y = y+l;
    points[2].z = 0;

    points[3].x = x-w;
    points[3].y = y+l;
    points[3].z = 0;

    points[4]=points[0];

    marker.points = points;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "fake_object_publisher");
    ros::NodeHandle nh;

    ros::Publisher chatter_pub = nh.advertise<custom_msgs::ObjectArray>("ObjectArray", 1);
    ros::Publisher pub_vis = nh.advertise<visualization_msgs::MarkerArray>("objectvis", 1);
    custom_msgs::ObjectArray objs;
    visualization_msgs::MarkerArray obj_vis;

    int n = 1;
    FakeObject obj(0,0,1,1);

    double start = ros::Time::now().toSec() + 3;
    double end = ros::Time::now().toSec() + 10;
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        double now = ros::Time::now().toSec();
        objs.objs.clear();
        obj_vis.markers.clear();
        if(now>start && now<end){
            obj.reckon(1,1);
            objs.objs.push_back(obj.obj);
            obj_vis.markers.push_back(obj.marker);
            ROS_INFO("%f, %f",obj_vis.markers[0].points[0].x,obj.obj.x_pos);
        }
        chatter_pub.publish(objs);
        pub_vis.publish(obj_vis);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}



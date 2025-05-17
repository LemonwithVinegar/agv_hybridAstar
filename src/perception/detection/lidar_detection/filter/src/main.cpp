#include<ros/ros.h>
#include<custom_msgs/LidarRawObject.h>
#include<custom_msgs/LidarRawObjectArray.h>
#include<custom_msgs/LaneLine.h>
#include<custom_msgs/LaneLineArray.h>
#include<compute/map.h>
#include<compute/frenet.h>

ros::Publisher filter_pub;
custom_msgs::LaneLineArray lane_line;

void Callback(const custom_msgs::LidarRawObjectArray &no_filter_obj)
{
    custom_msgs::LidarRawObjectArray filter_obj;
    if(!lane_line.lines.size())
        return;
    for(int i = 0; i < no_filter_obj.objs.size(); i++)
    {

        std::vector<double> object1 = getFrenet(no_filter_obj.objs[i].x_pos,no_filter_obj.objs[i].y_pos,lane_line.lines[0].x,lane_line.lines[0].y);
        std::vector<double> object2 = getFrenet(no_filter_obj.objs[i].x_pos,no_filter_obj.objs[i].y_pos,lane_line.lines[1].x,lane_line.lines[1].y);  
        if(object1[1] < 0 || object2[1] > 0) //初步过滤
            continue;
        filter_obj.objs.push_back(no_filter_obj.objs[i]);
    }
    filter_obj.head.frame_id = no_filter_obj.head.frame_id;
    filter_obj.head.stamp = no_filter_obj.head.stamp;
    filter_pub.publish(filter_obj);
}

void lane_callback(const custom_msgs::LaneLineArray &lane_msg)
{
    lane_line.lines.clear();
    lane_line.lines.push_back(lane_msg.lines[1]);
    lane_line.lines.push_back(lane_msg.lines[2]);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "node_filter");
    ros::NodeHandle nh;
    filter_pub = nh.advertise<custom_msgs::LidarRawObjectArray>("/detect_topic", 1, true);
    ros::Subscriber refer_lane_sub;
    refer_lane_sub = nh.subscribe("/road_lane", 1, lane_callback); //订阅车道线
    ros::Subscriber no_filter_detect_sub;
    no_filter_detect_sub = nh.subscribe("/no_filter_detect_topic", 1, Callback);  //订阅未过滤点
    ros::spin();
    return 0;
}

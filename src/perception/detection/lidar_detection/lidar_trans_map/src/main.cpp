#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf/transform_listener.h>
#include <custom_msgs/LidarRawObject.h>
#include <custom_msgs/LidarRawObjectArray.h>
ros::Publisher lidar_pub;
tf2_ros::Buffer *tf_buffer_ptr;
tf2_ros::TransformListener *tf_listener_ptr;
void lidar_map_transform(custom_msgs::LidarRawObject &LidarRawObject, const Eigen::Isometry3d &trans_lidar)
{
    Eigen::MatrixXd lidar_points;
    lidar_points = Eigen::MatrixXd::Ones(4,9);
    for (int i = 0;i < 8; i++)
    {
        lidar_points(0,i) = LidarRawObject.bbox_point[i].x;
        lidar_points(1,i) = LidarRawObject.bbox_point[i].y;
        lidar_points(2,i) = LidarRawObject.bbox_point[i].z;
        //lidar_points(2,i) = 0;
    }
    lidar_points(0,8) = LidarRawObject.x_pos;
    lidar_points(1,8) = LidarRawObject.y_pos;
    lidar_points(2,8) = LidarRawObject.z_pos;
    lidar_points = trans_lidar.matrix()*lidar_points;
    for(int i =0;i < 8; i++)
    {
        LidarRawObject.bbox_point[i].x = lidar_points(0,i);
        LidarRawObject.bbox_point[i].y = lidar_points(1,i);
        LidarRawObject.bbox_point[i].z = lidar_points(2,i);
    }
    LidarRawObject.x_pos = lidar_points(0,8);
    LidarRawObject.y_pos = lidar_points(1,8);
    LidarRawObject.z_pos = lidar_points(2,8);
}    

void Callback(const custom_msgs::LidarRawObjectArray &lidar_msg)
{
    geometry_msgs::TransformStamped transformStamped;
    Eigen::Isometry3d trans_lidar;
    custom_msgs::LidarRawObjectArray LidarRawObjects;
    //获取lidar到map的转换矩阵
    try
    {
        transformStamped = tf_buffer_ptr->lookupTransform("map", "lidar",ros::Time(0));
    }
    catch (tf::TransformException &ex)
    {
        ROS_WARN("%s",ex.what());
        return ;
    }
    trans_lidar = tf2::transformToEigen(transformStamped);
    for(int i = 0; i < lidar_msg.objs.size(); i++)
    {
        custom_msgs::LidarRawObject LidarRawObject;
        for(int j = 0; j < 8 ; j++)
        {
            LidarRawObject.bbox_point[j].x = lidar_msg.objs[i].bbox_point[j].x;
            LidarRawObject.bbox_point[j].y = lidar_msg.objs[i].bbox_point[j].y;
            LidarRawObject.bbox_point[j].z = lidar_msg.objs[i].bbox_point[j].z;
        }
        LidarRawObject.lwh.x = lidar_msg.objs[i].lwh.x;
        LidarRawObject.lwh.y = lidar_msg.objs[i].lwh.y;
        LidarRawObject.lwh.z = lidar_msg.objs[i].lwh.z;
        LidarRawObject.x_pos = lidar_msg.objs[i].x_pos;
        LidarRawObject.y_pos = lidar_msg.objs[i].y_pos;
        LidarRawObject.z_pos = lidar_msg.objs[i].z_pos;
        lidar_map_transform(LidarRawObject, trans_lidar);
        LidarRawObjects.objs.push_back(LidarRawObject);
    }
    LidarRawObjects.head.stamp = lidar_msg.head.stamp;
    LidarRawObjects.head.frame_id = "map";
    lidar_pub.publish(LidarRawObjects);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_to_map");
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);
    tf_buffer_ptr = &tf_buffer;
    tf_listener_ptr = &tf_listener;
    ros::NodeHandle nh;
    ros::Subscriber lidar_local = nh.subscribe("/lidar_detect_topic", 1, Callback);
    lidar_pub = nh.advertise<custom_msgs::LidarRawObjectArray>("/no_filter_detect_topic", 1, true);
    ros::spin();
    return 0;
}

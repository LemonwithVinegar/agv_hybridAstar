#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>

ros::Publisher pub;

uint16_t rebeam(uint16_t &beam) {
  switch (beam) {
  case 0:
    return 29;
  case 1:
    return 10;
  case 2:
    return 27;
  case 3:
    return 11;
  case 4:
    return 5;
  case 5:
    return 12;
  case 6:
    return 4;
  case 7:
    return 13;
  case 8:
    return 3;
  case 9:
    return 6;
  case 10:
    return 2;
  case 11:
    return 7;
  case 12:
    return 1;
  case 13:
    return 8;
  case 14:
    return 0;
  case 15:
    return 9;
  case 16:
    return 31;
  case 17:
    return 18;
  case 18:
    return 30;
  case 19:
    return 19;
  case 20:
    return 28;
  case 21:
    return 20;
  case 22:
    return 26;
  case 23:
    return 21;
  case 24:
    return 22;
  case 25:
    return 14;
  case 26:
    return 23;
  case 27:
    return 15;
  case 28:
    return 24;
  case 29:
    return 16;
  case 30:
    return 25;
  case 31:
    return 17;
  }
  return 0;
}

void chatterCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr rawCloudIn(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *rawCloudIn);
    pcl::PointCloud<pcl::PointXYZI>::Ptr sub_cloud(new pcl::PointCloud<pcl::PointXYZI>);
//    for(uint16_t j = 0;j<32;j++){
//        int offset = j*2028;
//        if(rebeam(j)>15)
//            sub_cloud->points.insert(sub_cloud->points.end(),
//                                     rawCloudIn.points.begin()+offset,
//                                     rawCloudIn.begin()+offset+2028);
//    }
    // 创建滤波对象
    pcl::VoxelGrid<pcl::PointXYZI> filter;
    filter.setInputCloud(rawCloudIn);
    // 设置体素栅格的大小为 1x1x1cm
    filter.setLeafSize(0.20f, 0.20f, 0.20f);
    sensor_msgs::PointCloud2 groundless_msg;
    filter.filter(*sub_cloud);
    pcl::toROSMsg(*sub_cloud, groundless_msg);
    groundless_msg.header.stamp = msg->header.stamp;
    groundless_msg.header.frame_id = msg->header.frame_id;
    pub.publish(groundless_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "points_converter");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("lidar_on_roof", 1, chatterCallback);
    pub = nh.advertise<sensor_msgs::PointCloud2>("points_raw", 1);
    ros::spin();

    return 0;
}

#include <string>
#include <vector>
#include <ros/ros.h>
#include "ros/package.h"
#include <stdio.h>
#include <stack>
#include <math.h>
#include <time.h>
#include <pcl/common/common.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include "cluster.h"
#include "boxer.h"
#include <Eigen/Core>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/console/time.h>
#include "tracker.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <custom_msgs/LidarRawObject.h>
#include <custom_msgs/LidarRawObjectArray.h>
ros::NodeHandle *nh;
ros::Publisher points_pub, raw_points_pub , pub_LidarRawObject;
image_transport::Publisher outmap_pub;
PolarGridBase *PolarGridBasor;
BoundingBoxCalculator *boxer;
Track *Trackor;
Eigen::Matrix3f car_pose_;
cv::Mat obj_image;


// std::vector<double> lane_start_sd, lane_end_sd;

void Callback(const sensor_msgs::PointCloud2ConstPtr &points_msg)
{

    clock_t start, end;
    start = clock();
    pcl::console::TicToc tt;
    tt.tic();
    double frame_time = points_msg->header.stamp.toSec();
    std::cout.precision(20);
    // std::cout<<"the points time is "<<frame_time<<std::endl;

    pcl::PointCloud<pcl::PointXYZI>::Ptr noground_cloud(new pcl::PointCloud<pcl::PointXYZI>);    

    pcl::fromROSMsg(*points_msg, *noground_cloud);

    std::vector<pcl::PointCloud<pcl::PointXYZI> > clusters;
    pcl::PointCloud<pcl::PointXYZRGB> color_cloud;
    PolarGridBasor->polarGridCluster(noground_cloud, clusters);

    std::vector<BoundingBoxCalculator::BoundingBox> boxes;
    std::vector<tracker> trackers;
    custom_msgs::LidarRawObjectArray LidarRawObjects;

    //simple classify the object
    for (int i = 0; i < clusters.size(); ++i)
    {
        BoundingBoxCalculator::BoundingBox tmp_box = boxer->calcBoundingBox(clusters[i]);
       if (tmp_box.size.x * tmp_box.size.y > 18)
           continue;
        Eigen::Vector2f v1(tmp_box.center.x, tmp_box.center.y);
        float distance = v1.norm();
        // if (tmp_box.size.z > 2.5 && distance > 8)
        //     continue;

       if (tmp_box.size.x / tmp_box.size.y > 4 && tmp_box.size.y < 0.1)
           continue;
       if (tmp_box.size.z / tmp_box.size.y > 8 && tmp_box.size.y < 0.2)
           continue;
       if (tmp_box.size.x < 0.4 && tmp_box.size.y < 0.4 && tmp_box.size.z < 0.4)
           continue;
       if(tmp_box.size.x < 0.1 || tmp_box.size.z < 0.1)
           continue;

        //boxes.push_back(tmp_box);
        tracker tmp_track;
        tmp_track.kalman_init();
        tmp_track.num_points = clusters[i].points.size();
        tmp_track.center[0] = tmp_box.center.x;
        tmp_track.center[1] = tmp_box.center.y;
        tmp_track.center[2] = 0;
        tmp_track.size.x = tmp_box.size.x;
        tmp_track.size.y = tmp_box.size.y;
        tmp_track.size.z = tmp_box.size.z;
        tmp_track.corners[0] = tmp_box.corners[0];
        tmp_track.corners[1] = tmp_box.corners[1];
        tmp_track.corners[2] = tmp_box.corners[2];
        tmp_track.corners[3] = tmp_box.corners[3];
        tmp_track.corners[4] = tmp_box.corners[4];
        tmp_track.corners[5] = tmp_box.corners[5];
        tmp_track.corners[6] = tmp_box.corners[6];
        tmp_track.corners[7] = tmp_box.corners[7];
        tmp_track.latest_tracked_time = frame_time;
        trackers.push_back(tmp_track);
        custom_msgs::LidarRawObject LidarRawObject;
        for(int i = 0; i < 8 ; i++)
        {
            LidarRawObject.bbox_point[i].x = tmp_box.corners[i].x;
            LidarRawObject.bbox_point[i].y = tmp_box.corners[i].y;
            LidarRawObject.bbox_point[i].z = tmp_box.corners[i].z;
        }
        LidarRawObject.lwh.x = tmp_box.size.x;
        LidarRawObject.lwh.y = tmp_box.size.y;
        LidarRawObject.lwh.z = tmp_box.size.z;
        LidarRawObject.x_pos = tmp_box.center.x;
        LidarRawObject.y_pos = tmp_box.center.y;
        LidarRawObject.z_pos = tmp_box.center.z;
        LidarRawObjects.objs.push_back(LidarRawObject);
    }
    LidarRawObjects.head = points_msg->header;
    pub_LidarRawObject.publish(LidarRawObjects);
//    Trackor->setNewObjects(trackers,frame_time);
//    std::vector<tracker> new_trackers;
//    Trackor->getObjects(new_trackers);
//    std::cout<<"the size of succeed_trackers is "<<new_trackers.size()<<std::endl;

    PolarGridBasor->clusterstoColor(clusters, color_cloud);
    
    
    sensor_msgs::PointCloud2 output_points;
    pcl::toROSMsg(color_cloud, output_points);
    output_points.header.frame_id = "rslidar";
    points_pub.publish(output_points);


//    std::cout << "the cost time of one frame is " << tt.toc() << std::endl;s

    std::vector<pcl::PointCloud<pcl::PointXYZI> >().swap(clusters);
    std::vector<BoundingBoxCalculator::BoundingBox>().swap(boxes);
    end = clock();
    float runTime = ((float)(end - start)) / CLOCKS_PER_SEC;
    //ROS_INFO("cluster time: %f seconds\n", runTime);
}

void object_cb(const sensor_msgs::ImageConstPtr &image_msg)
{
    std::vector<tracker> new_objects;
    Trackor->getObjects(new_objects);
    Trackor->getCarPose(car_pose_);

    const int FRONT_SCAN_RANGE = 250;      //前方扫描有效范围250
    const int LEFT_SCAN_RANGE = 100;       //左方扫描有效范围100

    obj_image = cv_bridge::toCvShare(image_msg, "bgr8")->image;
    cv::Point pt[1][4];

    for(int i = 0;i<new_objects.size();++i)
    {
        //new_objects[i].center = car_pose_.inverse()*new_objects[i].center;
        if(new_objects[i].center[0] > 20||new_objects[i].center[0] < -20)
            continue;
        if(new_objects[i].center[1] > 10||new_objects[i].center[1] < -10)
            continue;
        pt[0][0].y = FRONT_SCAN_RANGE - new_objects[i].corners[0].x/0.2;
        pt[0][0].x = LEFT_SCAN_RANGE - new_objects[i].corners[0].y/0.2;
        pt[0][1].y = FRONT_SCAN_RANGE - new_objects[i].corners[1].x/0.2; 
        pt[0][1].x = LEFT_SCAN_RANGE - new_objects[i].corners[1].y/0.2;             
        pt[0][2].y = FRONT_SCAN_RANGE - new_objects[i].corners[2].x/0.2;
        pt[0][2].x = LEFT_SCAN_RANGE - new_objects[i].corners[2].y/0.2;
        pt[0][3].y = FRONT_SCAN_RANGE - new_objects[i].corners[3].x/0.2; 
        pt[0][3].x = LEFT_SCAN_RANGE - new_objects[i].corners[3].y/0.2;     

        const cv::Point* ppt[1]={pt[0]};
        int npt[] = {4};
        cv::polylines(obj_image, ppt, npt, 1, 1, cv::Scalar(0,255,0),1,8,0);        
        cv::fillPoly(obj_image, ppt, npt, 1, cv::Scalar(0,255,0));     //绘制四边形,并填充
    }
    //std::cout<<new_objects.size()<<std::endl;


    // cv::namedWindow("object Image", cv::WINDOW_NORMAL);
    // cv::imshow("object Image",obj_image);
    // cv::waitKey(10);

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", obj_image).toImageMsg();
    outmap_pub.publish(msg);
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rs_localization");

    nh = new ros::NodeHandle;
    ros::NodeHandle pri_nh("~");
    float planefit_thre;

    std::string sub_no_ground_points_topic;
    pri_nh.param<std::string>("sub_no_ground_points_topic", sub_no_ground_points_topic,"/points_no_ground");

    std::string curb_topic_;
    pri_nh.param<std::string>("curb_topic_sub_", curb_topic_,"/grid_map");
    
    std::string pub_ground_points_topic;
    pri_nh.param<std::string>("pub_ground_points_topic", pub_ground_points_topic,"/points_cluster");

    std::string pub_raw_points_topic;
    pri_nh.param<std::string>("pub_raw_points_topic", pub_raw_points_topic,"/raw_point");

    //image publish init
    std::string outmap_topic;
    pri_nh.param<std::string>("out_map", outmap_topic, "/out_map");   

     //发布检测数据
    std::string pub_detect_topic;
    pri_nh.param<std::string>("pub_detect_topic", pub_detect_topic, "/lidar_detect_topic"); 

    PolarGridBasor = new PolarGridBase();
    Trackor = new Track(*nh);
    boxer = new BoundingBoxCalculator;
    boxer->init();

    points_pub = nh->advertise<sensor_msgs::PointCloud2>(pub_ground_points_topic, 1, true);
    pub_LidarRawObject = nh->advertise<custom_msgs::LidarRawObjectArray>(pub_detect_topic,1,true);
    image_transport::ImageTransport it(pri_nh);
    outmap_pub = it.advertise(outmap_topic, 1);
    //raw_points_pub = nh->advertise<sensor_msgs::PointCloud2>(pub_raw_points_topic, 1, true);

    ros::Subscriber points_sub;
    points_sub = nh->subscribe(sub_no_ground_points_topic, 1, Callback);  //订阅非地面点


    // ros::Subscriber car_pose_sub;
    // car_pose_sub = nh->subscribe("/cur_pose", 1, get_car_position);  //订阅车辆位姿
    
    // ros::Subscriber objects_node_sub_;
    // objects_node_sub_ = nh->subscribe(curb_topic_, 1, object_cb);


    //时间同步
    // message_filters::Subscriber<sensor_msgs::PointCloud2> points_sub(*nh, sub_points_topic, 1);
    // message_filters::Subscriber<sensor_msgs::Image> objects_node_sub_(*nh, curb_topic_, 1);
    // typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image> MySyncPolicy;
    // message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(100000), points_sub, objects_node_sub_);
    // sync.registerCallback(boost::bind(&Callback, _1, _2));

    ros::spin();
    return 0;
}

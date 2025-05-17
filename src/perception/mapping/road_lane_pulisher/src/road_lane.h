#ifndef ROAD_LANE_H
#define ROAD_LANE_H

#include <ros/ros.h>
#include <custom_msgs/LaneLine.h>
#include <custom_msgs/LaneLineArray.h>
#include <custom_msgs/CurPose.h>
#include <custom_msgs/Map_Switch.h>
#include <compute/map.h>
#include <compute/frenet.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Geometry>
#include <geometry_msgs/PoseArray.h>
#include <custom_msgs/CarDirection.h>

typedef struct inter_section
{
    bool IsInterSection; 
    double longitude;               
    double latitude;                  
    double longitude_stop_line;       
    double latitude_stop_line;        
}InterSection;

typedef struct tunnel
{
    bool IsTunnel;  
}Tunnel;

typedef struct current_lane  //获取车道线信息
{
    int nextlane;
    std::vector<LanePoint> left_road_points;
    std::vector<LanePoint> right_road_points;
    std::vector<LanePoint> refer_road_points;
    double width;
    LanePoint start_points;
    LanePoint end_points;
    InterSection pass_inter_section;
    Tunnel pass_tunnel;



}CurLane;

class RoadLane
{
public:
    RoadLane();
    int exec();
private:
    ros::NodeHandle nh;
    ros::Subscriber cur_pose_sub, running_status_sub,searched_points_sub;
    ros::Publisher road_lane_pub, cur_pose_sd_pub, cur_scene_pub, velocity_pub;
    ros::Publisher pub_s_dir;
    ros::ServiceClient turn_light_client;
    ros::ServiceClient avoid_ambulance_client;
    Lane_Message cur_lane;
    //int turn_light_num, map_type = 0;
    int last_map_type = 0;
    custom_msgs::LaneLine refer_road_lane,refer_points;
    custom_msgs::LaneLine left_road_lane, right_road_lane;
    geometry_msgs::TransformStamped transformStamped;  
    geometry_msgs::Pose2D slam_pose;
    //每条道路起点s
    std::vector<double> s_start;
    //每条道路终点s
    std::vector<double> s_end;

    ros::ServiceServer map_srv;
    bool map_switch(custom_msgs::Map_Switch::Request &req,
                    custom_msgs::Map_Switch::Response &res);

    bool TransformToMatrix(const geometry_msgs::TransformStamped& trans, Eigen::Matrix4f& transform_matrix);

    vector<LanePoint> LanePoints;

    bool IsInit, IsGetAvoidLane;
    void CurPoseRecvd(const geometry_msgs::Pose2D &msg);
    void onLaneLineArrayMsgRecvd(const geometry_msgs::PoseArray::ConstPtr& msg);
    void GetRoadLane(const int &lane_id);
    void IsLaneInit(const geometry_msgs::Pose2D &msg);
    void getAllLaneBEP(int laneCnt);
    void slam2gnssmap(const geometry_msgs::Pose::ConstPtr &msg);


    void GetAoidRoadLane();
    void init_s_size(int);
    void cal_s_value(int);

};



#endif 
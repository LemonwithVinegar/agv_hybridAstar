#include "rrt_star_global_planner/rrt_star_ros.h"
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/transform_listener.h>
#include <iostream>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <mutex> 

// 增加部分
#include "rrt_star_global_planner/gnss2slam.h"
#include "custom_msgs/Astar.h"
#include "custom_msgs/VehicleStat.h"
#include "geometry_msgs/Pose2D.h"
#include <Eigen/Core>
#include <custom_msgs/VehicleStat.h>
#include <geometry_msgs/PoseArray.h>
typedef typename Eigen::Vector3d Vec3d;

namespace RRTstar_planner
{

class RRTstarPlannerWithCostmap : public RRTstarPlannerROS
{
    public:
        RRTstarPlannerWithCostmap(std::string name, costmap_2d::Costmap2DROS* cmap);
        ~RRTstarPlannerWithCostmap();

    private:
        void MakePlan();
        void SetGoal(const geometry_msgs::PoseStamped::ConstPtr& goal);
        void SetStart(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& start);
        costmap_2d::Costmap2DROS* cmap_;
        ros::Subscriber pose_sub_;
        ros::Subscriber start_pose_sub_;

        geometry_msgs::PoseStamped start_;
        geometry_msgs::PoseStamped goal_;
        bool start_received_ = false;
        bool goal_received_ = false;
        bool cur_pose_received_ = false;

        std::mutex start_mutex_;
        std::mutex goal_mutex_;
        std::mutex cur_pose_mutex_;

        // 增加部分
        ros::Subscriber cur_pose_sub_;
        // ros::Subscriber goal_pose_sub_;
        ros::Subscriber stop_car_status_sub_;
        Vec3d cur_pose_;
        bool isStopCar = false;
        std::vector<geometry_msgs::PoseStamped> plan;  //搜索得到的路径点
        void onCurPoseMsgRecvd(const geometry_msgs::Pose2D::ConstPtr &msg);
        void stopCarStatusRecvd(const custom_msgs::VehicleStat::ConstPtr &msg);
        void PublishPath(); //发布路径点->定位，然后定位->规划
        ros::Publisher path_points_pub_;
        double path_id = 0; //每一段路径id

};


RRTstarPlannerWithCostmap::RRTstarPlannerWithCostmap(std::string name, costmap_2d::Costmap2DROS* cmap) :
        RRTstarPlannerROS(name, cmap)
{
    // ros::NodeHandle private_nh("move_base_simple");
    ros::NodeHandle private_nh;
    cmap_ = cmap;

    // 增加-------------
    cur_pose_sub_ = private_nh.subscribe<geometry_msgs::Pose2D>("/cur_pose", 1, &RRTstarPlannerWithCostmap::onCurPoseMsgRecvd, this);
    path_points_pub_ = private_nh.advertise<geometry_msgs::PoseArray>("searched_points", 1, true);//混合A*搜索的轨迹点发布
    // 增加-------------

    
    pose_sub_ = private_nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, &RRTstarPlannerWithCostmap::SetGoal, this);
    start_pose_sub_ = private_nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, &RRTstarPlannerWithCostmap::SetStart, this);



   
    
    stop_car_status_sub_ = private_nh.subscribe<custom_msgs::VehicleStat>("/stop_stat", 1, &RRTstarPlannerWithCostmap::stopCarStatusRecvd, this);//订阅一个停车状态标志位，判断是否发送下一段路径
}

RRTstarPlannerWithCostmap::~RRTstarPlannerWithCostmap()
{}


void RRTstarPlannerWithCostmap::MakePlan() {
  std::lock_guard<std::mutex> start_lock(start_mutex_);
  std::lock_guard<std::mutex> goal_lock(goal_mutex_);
  std::lock_guard<std::mutex> cur_pose_lock(cur_pose_mutex_);
  boost::unique_lock<costmap_2d::Costmap2D::mutex_t> map_lock(
      *(costmap_ros_->getCostmap()->getMutex()));


  if ((!cur_pose_received_ && !start_received_) || !goal_received_) {
    return;
  }
  goal_received_ =false;

  // std::vector<geometry_msgs::PoseStamped> plan;
  plan.clear();
  if (makePlan(start_, goal_, plan)) {
    // LOG(INFO) << "Successfully find a grid path via A* algorithm.";
    ROS_INFO("Successfully find a grid path via RRT algorithm.");
  } else {
    // LOG(ERROR) << "Failed to find a grid path via RRT algorithm.";
     ROS_INFO("Failed to find a grid path via RRT algorithm.");
  }
  PublishPath();
}

void RRTstarPlannerWithCostmap::PublishPath() {
// std::vector<geometry_msgs::PoseStamped> plan;
  geometry_msgs::PoseArray pose_array;
  geometry_msgs::Pose pose1;
  path_id++;  //从1开始计算
  for (const auto& poseStamped : plan) {
    pose1.position.x = poseStamped.pose.position.x;
    pose1.position.y = poseStamped.pose.position.y;
    pose1.position.z = path_id; //每一段路径的id,从1开始
    pose1.orientation.x = 1;   //每一段路径的方向
    pose1.orientation.y = 1;  //如果规划出的路径只有一段，1表示最后一段的标志位，用于最终的停车
    pose_array.poses.push_back(pose1);
    path_points_pub_.publish(pose_array);
  }
  pose_array.poses.clear();
     

}


void RRTstarPlannerWithCostmap::SetStart(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& start) {

    // double x = start->pose.pose.position.x;
    // double y = start->pose.pose.position.y;
    // // x= -49.49;
    // // y = 3.15;
    // std::cout<<"x = "<<x<<std::endl;

    // robot_pose.pose.position.x = x;
    // robot_pose.pose.position.y = y;

  {
    std::lock_guard<std::mutex> start_lock(start_mutex_);
    // LOG(INFO) << "A new start is received.";
    ROS_INFO("A new start is received.");
    start_.header = start->header;
    start_.pose = start->pose.pose;
    start_received_ = true;
  }

  MakePlan();
}

void RRTstarPlannerWithCostmap::SetGoal(const geometry_msgs::PoseStamped::ConstPtr& goal)
{

    unsigned int mx = 0,my = 0;
    if(!this->costmap_->worldToMap(goal->pose.position.x,goal->pose.position.y,mx,my))
    {
      std::cout << "worldToMap error" << std::endl;
      return;
    }
    if(this->costmap_->getCost(mx,my) != costmap_2d::FREE_SPACE)
    {
      std::cout << "The target point is unreachable." << std::endl;
      return;
    }
    {
        std::lock_guard<std::mutex> goal_lock(goal_mutex_);
        // LOG(INFO) << "A new goal is received.";
        ROS_INFO("A new goal is received.");
        goal_ = *goal;
        goal_received_ = true;
    }
    MakePlan();
}


void RRTstarPlannerWithCostmap::onCurPoseMsgRecvd(const geometry_msgs::Pose2D::ConstPtr &msg) {
    // 转换成slam坐标，目前是cur_pose转到slam
    geometry_msgs::Pose2D aft_pose2d;  
    aft_pose2d = gnss2slam(msg);
    cur_pose_ = Vec3d(      
            aft_pose2d.x,
            aft_pose2d.y,
            aft_pose2d.theta
    );  
    // ROS_INFO_STREAM_THROTTLE(1,"x = : "<< aft_pose2d.x);
    // std::cout<< "x: " <<aft_pose2d.x<<"y: " <<aft_pose2d.y<<std::endl;
    {
      std::lock_guard<std::mutex> cur_pose_lock(cur_pose_mutex_);
      // LOG(INFO) << "A new start is received.";
      // ROS_INFO("A new start is received.");

      start_.pose.position.x = aft_pose2d.x;
      start_.pose.position.y = aft_pose2d.y;
      // start_.pose.position.x = -41.682964325;
      // start_.pose.position.y = -19.2029685974;
      cur_pose_received_ = true;
    }
    if (goal_received_) {
      MakePlan();
    }
    
}

void RRTstarPlannerWithCostmap::stopCarStatusRecvd(const custom_msgs::VehicleStat::ConstPtr &msg) {
    isStopCar = msg->isStopCar;
    // cout<<"isStopCar "<<isStopCar<<endl;
}



} // namespace

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rrt_star_planner");
    tf2_ros::Buffer buffer(ros::Duration(10));
    tf2_ros::TransformListener tf(buffer);
    costmap_2d::Costmap2DROS gcm("global_costmap", buffer);
    RRTstar_planner::RRTstarPlannerWithCostmap pppp("RRTstarPlannerROS", &gcm);
    ros::spin();
    return 0;
}


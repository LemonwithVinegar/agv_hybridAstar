#include <ros/package.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <fstream>
#include <string>
#include "rrt_star_global_planner/gnss2slam.h"
#include <tf/transform_datatypes.h>

//读取points.txt中的目标点发布，x y theta

int main(int argc, char** argv)
{
    ros::init(argc, argv, "goal_publisher");
    ros::NodeHandle nh;

    std::string file_path = ros::package::getPath("rrt_star_global_planner") + "/data/points.txt";
    std::ifstream file;

    ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);

    while (ros::ok()) {
        file.open(file_path);
        if (file.is_open()) {
            std::string line;
            while (std::getline(file, line)) {
                std::stringstream ss(line);
                double x, y, theta;
                if (ss >> x >> y >> theta) {
                    geometry_msgs::PoseStamped pose_msg;
                    geometry_msgs::Pose2D pose;
                    geometry_msgs::Pose2D aft_pose2d;
                    //gnss坐标
                    pose.x = x;
                    pose.y = y;
                    pose.theta = theta;
                    geometry_msgs::Pose2D::ConstPtr pose_ptr = boost::make_shared<const geometry_msgs::Pose2D>(pose);
                    aft_pose2d = gnss2slam(pose_ptr);//gnss坐标转slam


                    pose_msg.header.stamp = ros::Time::now();
                    pose_msg.header.frame_id = "map";


                    // pose_msg.pose.position.x = x;
                    // pose_msg.pose.position.y = y;
                    // pose_msg.pose.position.z = theta;
                    
                    pose_msg.pose.position.x = aft_pose2d.x;
                    pose_msg.pose.position.y = aft_pose2d.y;
                    pose_msg.pose.position.z = 0;
                    // 使用偏航角创建四元数（geometry_msgs::Quaternion）
                    geometry_msgs::Quaternion quat_msg = tf::createQuaternionMsgFromYaw(aft_pose2d.theta);
                    // 将 geometry_msgs::Quaternion 转换为 tf::Quaternion
                    tf::Quaternion quat;
                    tf::quaternionMsgToTF(quat_msg, quat);
                    pose_msg.pose.orientation.x = quat.x();
                    pose_msg.pose.orientation.y = quat.y();
                    pose_msg.pose.orientation.z = quat.z();
                    pose_msg.pose.orientation.w = quat.w();
                    pub.publish(pose_msg);
                    ROS_INFO_STREAM("Published goal pose: " << x << " " << y << " " << theta);
                    // remove the line from file
                    std::ofstream temp_file;
                    temp_file.open(file_path + ".temp");
                    while (std::getline(file, line)) {
                        temp_file << line << std::endl;
                    }
                    temp_file.close();
                    file.close();
                    std::remove(file_path.c_str());
                    std::rename((file_path + ".temp").c_str(), file_path.c_str());
                    break;
                }
            }
            file.close();
        }

        ros::spinOnce();
        ros::Duration(1).sleep();  // sleep for 1 seconds
    }

    return 0;
}



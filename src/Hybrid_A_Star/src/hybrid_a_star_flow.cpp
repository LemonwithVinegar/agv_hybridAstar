/*******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2022 Zhang Zhimeng
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
 * WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#include "hybrid_a_star/hybrid_a_star_flow.h"

#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "hybrid_a_star/gnss2slam.h"
#include "custom_msgs/Astar.h"
// #include <custom_msgs/VehicleStat.h>



double Mod2Pi(const double &x) {
    double v = fmod(x, 2 * M_PI);

    if (v < -M_PI) {
        v += 2.0 * M_PI;
    } else if (v > M_PI) {
        v -= 2.0 * M_PI;
    }

    return v;
}




HybridAStarFlow::HybridAStarFlow(ros::NodeHandle &nh) {
    double steering_angle = nh.param("planner/steering_angle", 10);
    int steering_angle_discrete_num = nh.param("planner/steering_angle_discrete_num", 1);
    double wheel_base = nh.param("planner/wheel_base", 1.0);
    double segment_length = nh.param("planner/segment_length", 1.6);
    int segment_length_discrete_num = nh.param("planner/segment_length_discrete_num", 8);
    double steering_penalty = nh.param("planner/steering_penalty", 1.05);
    double steering_change_penalty = nh.param("planner/steering_change_penalty", 1.5);
    double reversing_penalty = nh.param("planner/reversing_penalty", 2.0);
    double shot_distance = nh.param("planner/shot_distance", 5.0);
  
    kinodynamic_astar_searcher_ptr_ = std::make_shared<HybridAStar>(
            steering_angle, steering_angle_discrete_num, segment_length, segment_length_discrete_num, wheel_base,
            steering_penalty, reversing_penalty, steering_change_penalty, shot_distance
    );
    costmap_sub_ptr_ = std::make_shared<CostMapSubscriber>(nh, "/map", 1);  //订阅栅格地图
    simulate_goal_pose_sub_ptr_ = std::make_shared<SimulateGoalPoseSubscriber2D>(nh, "/move_base_simple/goal", 1);
    goal_pose_sub_ptr_ = std::make_shared<GoalPoseSubscriber2D>(nh, "/goal", 1);
    path_pub_ = nh.advertise<nav_msgs::Path>("searched_path", 1);//发布分段后的路径
    path_points_pub_ = nh.advertise<geometry_msgs::PoseArray>("searched_points", 1, true);//混合A*搜索的轨迹点发布
    searched_tree_pub_ = nh.advertise<visualization_msgs::Marker>("searched_tree", 1);
    vehicle_path_pub_ = nh.advertise<visualization_msgs::MarkerArray>("vehicle_path", 1);
    path_info_pub_ = nh.advertise<custom_msgs::Astar>("path_info", 1);  //发布一个标志位，用于解除上一次规划最后一段停车状态

    cur_pose_sub_ = nh.subscribe("/cur_pose", 1, &HybridAStarFlow::onCurPoseMsgRecvd, this);
    
    stop_car_status_sub_ = nh.subscribe("/stop_stat", 1, &HybridAStarFlow::stopCarStatusRecvd, this);//订阅一个停车状态标志位，判断是否发送下一段路径
    has_map_ = false;
}

void HybridAStarFlow::Run() {
    ReadData();

    if (!has_map_) {
        if (costmap_deque_.empty()) {
            return;
        }

        current_costmap_ptr_ = costmap_deque_.front();
        costmap_deque_.pop_front();

        const double map_resolution = 0.2;
    
        kinodynamic_astar_searcher_ptr_->Init( //origin[x,y,θ]：表示栅格坐标系与世界坐标系的x、y、角度偏差。其中x、y单位均为m
                current_costmap_ptr_->info.origin.position.x,//获得栅格地图的原点x值(相对世界坐标系),单位为m
                1.0 * current_costmap_ptr_->info.width * current_costmap_ptr_->info.resolution,
                current_costmap_ptr_->info.origin.position.y,//获得栅格地图的原点y值(相对世界坐标系),单位为m
                1.0 * current_costmap_ptr_->info.height * current_costmap_ptr_->info.resolution,
                current_costmap_ptr_->info.resolution,//获得栅格地图的分辨率
                map_resolution
        );

        unsigned int map_w = std::floor(current_costmap_ptr_->info.width *current_costmap_ptr_->info.resolution / map_resolution);
        unsigned int map_h = std::floor(current_costmap_ptr_->info.height *current_costmap_ptr_->info.resolution/ map_resolution);
        for (unsigned int w = 0; w < map_w; ++w) {
            for (unsigned int h = 0; h < map_h; ++h) {
                auto x = static_cast<unsigned int> ((w + 0.5) * map_resolution
                                                    / current_costmap_ptr_->info.resolution);
                auto y = static_cast<unsigned int> ((h + 0.5) * map_resolution
                                                    / current_costmap_ptr_->info.resolution);

                if (current_costmap_ptr_->data[y * current_costmap_ptr_->info.width + x]) {
                    kinodynamic_astar_searcher_ptr_->SetObstacle(w, h);
                }
            }
        }
        has_map_ = true;
    }
    costmap_deque_.clear();
    

    while (HasGoalPose() || HasSimulateGoalPose()) {   
        InitPoseData();
        double goal_yaw;
        Vec3d start_state = get_CurPose_WithMutex();
        
        if (HasSimulateGoalPose()) {
            goal_yaw = tf::getYaw(current_goal_pose_ptr_->pose.orientation);
            simulate_goal_pose_deque_.pop_front();
        } else {
            goal_yaw =  current_goal_pose_ptr_->pose.position.z;   
            goal_pose_deque_.pop_front();
            // std::cout << "1111111";     
        }
        Vec3d goal_state = Vec3d(
                current_goal_pose_ptr_->pose.position.x,
                current_goal_pose_ptr_->pose.position.y,
                goal_yaw
        );
        std::cout<< "goal.x:" <<current_goal_pose_ptr_->pose.position.x<< " "<<"goal.y:"<<current_goal_pose_ptr_->pose.position.y<<std::endl;
        // std::cout<< "rviz angle: "<< tf::getYaw(current_goal_pose_ptr_->pose.orientation) * 180.0/ M_PI  <<std::endl;
        ROS_INFO_STREAM_THROTTLE(1, "goal position - x: " << current_goal_pose_ptr_->pose.position.x << ", y: " << current_goal_pose_ptr_->pose.position.y << ", theta: " << goal_yaw);
        bool is_new_planning = false;  //执行新的一次规划标志
        if (kinodynamic_astar_searcher_ptr_->Search(start_state, goal_state)) {
            custom_msgs::Astar path_info;
            planning_count++;
            // std::cout<<"--flag: "<<flag<<std::endl;
            if (planning_count != 1) {
                is_new_planning = true;
            }
            // std::cout<<"plancount: "<<plancount<<std::endl;
            // std::cout<<"flag: "<<flag<<std::endl;
            path_info.new_plan_executing = is_new_planning;
            path_info_pub_.publish(path_info);
            auto path = kinodynamic_astar_searcher_ptr_->GetPath();
            PublishPath(path);

            is_new_planning = false;
    }
        // debug
        //std::cout << "visited nodes: " << kinodynamic_astar_searcher_ptr_->GetVisitedNodesNumber() << std::endl;
    kinodynamic_astar_searcher_ptr_->Reset();
    }
}

void HybridAStarFlow::ReadData() {
    costmap_sub_ptr_->ParseData(costmap_deque_);
    simulate_goal_pose_sub_ptr_->ParseData(simulate_goal_pose_deque_);
    goal_pose_sub_ptr_->ParseData(goal_pose_deque_);
}

void HybridAStarFlow::InitPoseData() {
    if(HasGoalPose()) {     //实车指定
        current_goal_pose_ptr_ = goal_pose_deque_.front();
        // goal_pose_deque_.pop_front();
    } else {                //仿真指定
        current_goal_pose_ptr_ = simulate_goal_pose_deque_.front();
        // simulate_goal_pose_deque_.pop_front();
    }  
}

bool HybridAStarFlow::HasGoalPose() {
    return !goal_pose_deque_.empty();
}

bool HybridAStarFlow::HasSimulateGoalPose() {
    return !simulate_goal_pose_deque_.empty();
}

// 通过PublishPath()函数发布路径点
void HybridAStarFlow::PublishPath(const VectorVec3d &  path) {
    nav_msgs::Path nav_path;
    geometry_msgs::PoseStamped pose_stamped;
    
    vector<vector<double>> trajectory;
    
    for (const auto &pose: path) {
        pose_stamped.header.frame_id = "slam_map";
        pose_stamped.pose.position.x = pose.x();
        pose_stamped.pose.position.y = pose.y();
        pose_stamped.pose.position.z = 0.0;
        pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(pose.z());

        trajectory.push_back({pose.x(), pose.y()});

        
        // std::cout<< pose.x()<<" "<<pose.y()<<std::endl;
        nav_path.poses.emplace_back(pose_stamped);
    }
    nav_path.header.frame_id = "slam_map";
    nav_path.header.stamp = timestamp_;
    
    path_pub_.publish(nav_path);
    // segments.clear();
    segments = segment_trajectory(trajectory);
    trajectory.clear();
    // path_points_pub_.publish(pose_array);
}

//没用到
void HybridAStarFlow::PublishVehiclePath(const VectorVec3d &path, double width,
                                         double length, unsigned int vehicle_interval = 5u) {
    visualization_msgs::MarkerArray vehicle_array;

    for (unsigned int i = 0; i < path.size(); i += vehicle_interval) {
        visualization_msgs::Marker vehicle;

        if (i == 0) {
            vehicle.action = 3;
        }

        vehicle.header.frame_id = "slam_map";
        vehicle.header.stamp = ros::Time::now();
        vehicle.type = visualization_msgs::Marker::CUBE;
        vehicle.id = static_cast<int>(i / vehicle_interval);
        vehicle.scale.x = width;
        vehicle.scale.y = length;
        vehicle.scale.z = 0.01;
        vehicle.color.a = 0.1;

        vehicle.color.r = 1.0;
        vehicle.color.b = 0.0;
        vehicle.color.g = 0.0;

        vehicle.pose.position.x = path[i].x();
        vehicle.pose.position.y = path[i].y();
        vehicle.pose.position.z = 0.0;

        vehicle.pose.orientation = tf::createQuaternionMsgFromYaw(path[i].z());
        vehicle_array.markers.emplace_back(vehicle);
    }

    vehicle_path_pub_.publish(vehicle_array);
}

void HybridAStarFlow::PublishSearchedTree(const VectorVec4d &searched_tree) {
    visualization_msgs::Marker tree_list;
    tree_list.header.frame_id = "slam_map";
    tree_list.header.stamp = ros::Time::now();
    tree_list.type = visualization_msgs::Marker::LINE_LIST;
    tree_list.action = visualization_msgs::Marker::ADD;
    tree_list.ns = "searched_tree";
    tree_list.scale.x = 0.02;

    tree_list.color.a = 1.0;
    tree_list.color.r = 0;
    tree_list.color.g = 0;
    tree_list.color.b = 0;

    tree_list.pose.orientation.w = 1.0;
    tree_list.pose.orientation.x = 0.0;
    tree_list.pose.orientation.y = 0.0;
    tree_list.pose.orientation.z = 0.0;

    geometry_msgs::Point point;
    for (const auto &i: searched_tree) {
        point.x = i.x();
        point.y = i.y();
        point.z = 0.0;
        tree_list.points.emplace_back(point);

        point.x = i.z();
        point.y = i.w();
        point.z = 0.0;
        tree_list.points.emplace_back(point);
    }

    searched_tree_pub_.publish(tree_list);
}

void HybridAStarFlow::onCurPoseMsgRecvd(const geometry_msgs::Pose2D::ConstPtr &msg) {
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
}

Vec3d HybridAStarFlow::get_CurPose_WithMutex() {
    Vec3d res;  
    cur_pose_mutex_.lock();
    res = cur_pose_;
    cur_pose_mutex_.unlock(); 
    return res; 
}

void HybridAStarFlow::stopCarStatusRecvd(const custom_msgs::VehicleStat::ConstPtr &msg) {
    isStopCar = msg->isStopCar;
    // cout<<"isStopCar "<<isStopCar<<endl;
}

// 在run_hybrid_astar.cpp中用多线程发布多段路径，解决消息回调时被阻塞在这段代码中的循环中
void HybridAStarFlow::publishPathSegmentsThread()
{   
    geometry_msgs::PoseArray pose_array;
    geometry_msgs::Pose pose1;
    int segments_size = getSegmentsSize();
    if (segments_size) {
        cout << "segments_size " << segments_size << endl;
        pose_array.poses.clear();
        ROS_INFO_STREAM_THROTTLE(1, "-----isStopCar: " << isStopCar);


        // for (int i = 0; i < segments.size(); i++) {
        //     cout << "Segment " << i + 1 << ": ";
        //     for (int j = 0; j < segments[i].first.size(); j+=2) {
        // }

        path_id++;  //从1开始计算
        //发布第一段路径
        for (size_t j = 0; j < segments[0].first.size(); j += 2) {
            // cout << "(" << segments[i].first[j] << ", " << segments[i].first[j + 1] << ") ";
            pose1.position.x = segments.at(0).first.at(j);
            pose1.position.y = segments.at(0).first.at(j + 1);
            pose1.position.z = path_id; //每一段路径的id,从1开始
            pose1.orientation.x = segments.at(0).second;   //每一段路径的方向
            if (segments_size == 1) {   //如果规划出的路径只有一段，1表示最后一段的标志位，用于最终的停车
                pose1.orientation.y = 1;
            } else {
                pose1.orientation.y = 0;
            }
            // ROS_INFO_STREAM_THROTTLE(1, "pose1.orientation.y: " << pose1.orientation.y);
            pose_array.poses.push_back(pose1);
            path_points_pub_.publish(pose_array);
            // goal_pose_sub_ptr_->SetFlag(false);
        }

        int i = 1;
        //发布后面几段路径
        while (i < segments_size) {

            // **********暂时没用*********
            // bool flag = goal_pose_sub_ptr_->GetFlag();
            // if (flag) {
            //     // simulate_goal_pose_sub_ptr_->SetFlag(false);
            //     goal_pose_sub_ptr_->SetFlag(false);
            //     kinodynamic_astar_searcher_ptr_->Reset(); 
            //     break;
            // }
            // **********暂时没用*********

            // for (int i = 1; i < segments.size(); i++) {
            ROS_INFO_STREAM_THROTTLE(1, "isStopCar = : " << isStopCar);
            //前一段路径执行完停下后，发布一个停车的标志位 isStopCar = 1继续发布后面的一段
            if (isStopCar) {
                // ROS_INFO_STREAM_THROTTLE(1, "isStopCar--- = : " << isStopCar);
                path_id++;  
                pose_array.poses.clear();
                cout<<"path_id  = : " << path_id<<endl;
                for (size_t j = 0; j < segments[i].first.size(); j += 2) {
                    pose1.position.x = segments.at(i).first.at(j);
                    pose1.position.y = segments.at(i).first.at(j + 1);
                    pose1.position.z = path_id;
                    pose1.orientation.x = segments.at(i).second;
                    if (i == (segments_size - 1)) {
                        pose1.orientation.y = 1;    //如果规划出的路径是最后一段，1表示最后一段的标志位，用于最终的停车
                    } else {
                        pose1.orientation.y = 0;
                    }
                    pose_array.poses.emplace_back(pose1);
                    path_points_pub_.publish(pose_array);   
                }
                isStopCar = false;
                i++;
                // cout<<"i = : " << i<<endl;
            }
        }
        ROS_INFO_STREAM_THROTTLE(1, "****************while finish********************");
    }

    pose_array.poses.clear();
    segments.clear();
    threadFinished = true;  
}



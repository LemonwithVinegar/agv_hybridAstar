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

#ifndef HYBRID_A_STAR_HYBRID_A_STAR_FLOW_H
#define HYBRID_A_STAR_HYBRID_A_STAR_FLOW_H

#include "hybrid_a_star.h"
#include "costmap_subscriber.h"
#include "goal_pose_subscriber.h"
#include "simulate_goal_pose_subscriber.h"


#include <mutex>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include "geometry_msgs/Pose2D.h"
#include <geometry_msgs/PoseArray.h>
#include"hybrid_a_star/segment_trajectory.h"

#include <custom_msgs/VehicleStat.h>
#include <atomic>


class HybridAStarFlow {
public:
    HybridAStarFlow() = default;

    explicit HybridAStarFlow(ros::NodeHandle &nh);

    void Run();
    int getSegmentsSize() const {
        return segments.size();
    }

    void publishPathSegmentsThread();
    bool isThreadFinished() const {
        return threadFinished;
    }
    void setThreadFinished(bool value) {
        threadFinished = value;
    }

private:
    void InitPoseData();

    void ReadData();

    bool HasSimulateGoalPose();

    bool HasGoalPose();

    void PublishPath(const VectorVec3d &path);

    void PublishSearchedTree(const VectorVec4d &searched_tree);

    void PublishVehiclePath(const VectorVec3d &path, double width,
                            double length, unsigned int vehicle_interval);

    void onCurPoseMsgRecvd(const geometry_msgs::Pose2D::ConstPtr &msg);
    void stopCarStatusRecvd(const custom_msgs::VehicleStat::ConstPtr &msg);

    Vec3d get_CurPose_WithMutex();

private:
    std::shared_ptr<HybridAStar> kinodynamic_astar_searcher_ptr_;
    std::shared_ptr<CostMapSubscriber> costmap_sub_ptr_;
    std::shared_ptr<SimulateGoalPoseSubscriber2D> simulate_goal_pose_sub_ptr_;
    std::shared_ptr<GoalPoseSubscriber2D> goal_pose_sub_ptr_;

    ros::Publisher path_pub_;
    ros::Publisher path_points_pub_;
    ros::Publisher searched_tree_pub_;
    ros::Publisher vehicle_path_pub_;
    ros::Publisher path_info_pub_;

    

    ros::Subscriber cur_pose_sub_;
    ros::Subscriber goal_pose_sub_;
    ros::Subscriber stop_car_status_sub_;

    std::deque<geometry_msgs::PoseStampedPtr> goal_pose_deque_;
    std::deque<geometry_msgs::PoseStampedPtr> simulate_goal_pose_deque_;
    std::deque<nav_msgs::OccupancyGridPtr> costmap_deque_;

    geometry_msgs::PoseStampedPtr current_goal_pose_ptr_;
    nav_msgs::OccupancyGridPtr current_costmap_ptr_;


    ros::Time timestamp_;

    bool has_map_{};

    std::mutex cur_pose_mutex_, goal_pose_mtx_;

    Vec3d cur_pose_, goal_pose_;

    bool isStopCar = false;

    int planning_count = 0;

    vector<pair<vector<double>, int>> segments;
    // vector<pair<vector<double>, int>> segments_copy;
    double path_id = 0; //每一段路径id
  
    std::atomic<bool> threadFinished; //线程完成标志，在构造函数中初始化为true

};

#endif //HYBRID_A_STAR_HYBRID_A_STAR_FLOW_H

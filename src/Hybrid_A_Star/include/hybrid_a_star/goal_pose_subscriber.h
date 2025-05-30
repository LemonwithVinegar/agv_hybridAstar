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

#ifndef HYBRID_A_STAR_GOAL_POSE_SUBSCRIBER_H
#define HYBRID_A_STAR_GOAL_POSE_SUBSCRIBER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <deque>
#include <mutex>

class GoalPoseSubscriber2D {
public:
    GoalPoseSubscriber2D(ros::NodeHandle &nh, const std::string &topic_name, size_t buff_size);

    void ParseData(std::deque<geometry_msgs::PoseStampedPtr> &pose_data_buff);

    bool GetFlag() const { return flag_; }  // 新增GetFlag函数
    void SetFlag(bool flag) { flag_ = flag; }
private:
    void MessageCallBack(const geometry_msgs::PoseStampedPtr &goal_pose_ptr);

private:
    ros::Subscriber subscriber_;
    std::deque<geometry_msgs::PoseStampedPtr> goal_poses_;

    std::mutex buff_mutex_;
    bool flag_ = false;  // 新增标志位
};



#endif //HYBRID_A_STAR_GOAL_POSE_SUBSCRIBER_H

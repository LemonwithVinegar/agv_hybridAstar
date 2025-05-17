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

#include "hybrid_a_star/simulate_goal_pose_subscriber.h"

SimulateGoalPoseSubscriber2D::SimulateGoalPoseSubscriber2D(ros::NodeHandle &nh,
                                           const std::string &topic_name,
                                           size_t buff_size) {
    subscriber_ = nh.subscribe(
            topic_name, buff_size, &SimulateGoalPoseSubscriber2D::MessageCallBack, this
    );
}

void SimulateGoalPoseSubscriber2D::MessageCallBack(
        const geometry_msgs::PoseStampedPtr &simulate_goal_pose_ptr
) {
    buff_mutex_.lock();
    simulate_goal_poses_.emplace_back(simulate_goal_pose_ptr);
    flag_ = true;  // 设置标志位为true
    buff_mutex_.unlock();
}

void SimulateGoalPoseSubscriber2D::ParseData(
        std::deque<geometry_msgs::PoseStampedPtr> &pose_data_buff
) {
    buff_mutex_.lock();
    if (!simulate_goal_poses_.empty()) {
        pose_data_buff.insert(pose_data_buff.end(), simulate_goal_poses_.begin(), simulate_goal_poses_.end());
        simulate_goal_poses_.clear();
    }
    buff_mutex_.unlock();
}
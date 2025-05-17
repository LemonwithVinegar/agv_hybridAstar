#ifndef GNSS_2_SLAM_H
#define GNSS_2_SLAM_H
#include "geometry_msgs/Pose2D.h"
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Dense>


geometry_msgs::TransformStamped transformStamped;

bool TransformToMatrix(const geometry_msgs::TransformStamped& trans, Eigen::Matrix4f& transform_matrix) {  

    double roll, pitch, yaw;
	tf::StampedTransform transform;
	//
	transform.setOrigin( tf::Vector3(trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z) );
	transform.setRotation( tf::Quaternion(trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w) );
    //平移 
    Eigen::Translation3f tl_btol(transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ());
    //姿态
    tf::Matrix3x3( transform.getRotation() ).getEulerYPR(yaw, pitch, roll); //ros中的角

    Eigen::AngleAxisf rot_x_btol( roll, Eigen::Vector3f::UnitX() );     //转为Eigen中的旋转向量， 以x轴旋转roll弧度
    Eigen::AngleAxisf rot_y_btol( pitch, Eigen::Vector3f::UnitY() ); 
    Eigen::AngleAxisf rot_z_btol( yaw, Eigen::Vector3f::UnitZ() ); 

    //TF变换转换到变换矩阵 
    transform_matrix = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();  //右乘

    return true; 
}



// 传入cur_pose，转到slam坐标系下去规划

geometry_msgs::Pose2D gnss2slam(const geometry_msgs::Pose2D::ConstPtr &msg)
{
    geometry_msgs::Pose2D aft_pose2d;
    Eigen::Matrix4f Tgo_so; 
    Eigen::Matrix3f rotationMatrix;
    Eigen::Vector3f euler_angles;

    // 转cur_pose
    // VehPose pose_gps(msg->longitude, msg->latitude, msg->heading);
    // trans_pose_gps2coord(map_ori, pose_gps, pos_map);

    // cur_pose.x = pos_map.x_lon;
    // cur_pose.y = pos_map.y_lat; 
    // cur_pose.theta = pos_map.yaw_heading;
    // theta = map_yaw - msg->heading;

    // 转slam坐标系
    if (1) {    // 已知gnss原点与slam原点的坐标系变换关系 （base_link in map）
        transformStamped.transform.translation.x = 1.9;
        transformStamped.transform.translation.y = -5.3;
        transformStamped.transform.translation.z = 0.0;

        transformStamped.transform.rotation.x = 0;  // xyz w
        transformStamped.transform.rotation.y = 0;
        transformStamped.transform.rotation.z = -0.982;
        transformStamped.transform.rotation.w = 0.191;		
    	}
    TransformToMatrix(transformStamped, Tgo_so); 

    Eigen::Matrix4f temp_pose = Eigen::Matrix4f::Identity();
    // temp_pose(0,3) = pos_map.x_lon;
    // temp_pose(1,3) = pos_map.y_lat; 
    temp_pose(0,3) = msg->x;
    temp_pose(1,3) = msg->y; 
    temp_pose(2,3) = 0.0;
    temp_pose.block<3,3>(0,0) = Eigen::AngleAxisf(msg->theta * M_PI / 180.0, Eigen::Vector3f(0,0,1)).toRotationMatrix();
    Eigen::Matrix4f aft_mat = Tgo_so.inverse() * temp_pose;

    // 取出对应的值
    aft_pose2d.x = aft_mat(0,3);
    aft_pose2d.y = aft_mat(1,3);
    rotationMatrix = aft_mat.block<3,3>(0,0);
    euler_angles = rotationMatrix.eulerAngles(2, 1, 0);

    // convert
    if (euler_angles(2)!=0) {   // 左右下半圈
            euler_angles(0) = -M_PI + euler_angles(0);
    } 

    aft_pose2d.theta  =  euler_angles(0);           // 弧度
    return aft_pose2d;
    // std::cout << "convert angle: " << aft_pose2d.theta * 180.0 / M_PI  << std::endl;    
}

#endif //GNSS_2_SLAM_H
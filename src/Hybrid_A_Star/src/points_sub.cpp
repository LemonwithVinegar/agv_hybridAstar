#include "ros/ros.h"
#include "std_msgs/String.h"
#include<geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
using namespace std;
#include<geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Geometry>
#include <geometry_msgs/PoseArray.h>
#include<geometry_msgs/Pose2D.h>
 geometry_msgs::TransformStamped transformStamped;  
    geometry_msgs::Pose2D slam_pose;

//开发过程中测试用

//转换矩阵获取：ros==>Eigen的接口 
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


void slam2gnssmap(const geometry_msgs::Pose::ConstPtr &msg)  	 
{
    geometry_msgs::TransformStamped  slamTrans; 
    Eigen::Matrix4f Tso_car, Tgo_car;
    Eigen::Matrix3f rotationMatrix;
    Eigen::Matrix4f Tgo_so; 
    Eigen::Vector3f euler_angles;

    if (1) {    // 已知gnss原点与slam原点的坐标系变换关系 （base_link in map）
        transformStamped.transform.translation.x = 0;
        transformStamped.transform.translation.y = 0;
        transformStamped.transform.translation.z = 0;

        transformStamped.transform.rotation.x = 0;  // xyz w
        transformStamped.transform.rotation.y = 0;
        transformStamped.transform.rotation.z = 0.97476;
        transformStamped.transform.rotation.w = 1;		
    	}
    TransformToMatrix(transformStamped, Tgo_so); 

    slamTrans.transform.translation.x = msg->position.x;
    slamTrans.transform.translation.y = msg->position.y;
    slamTrans.transform.translation.z = msg->position.z;
    slamTrans.transform.rotation = msg->orientation;
    
    //convert   position  and angle
    TransformToMatrix(slamTrans, Tso_car); 
    Tgo_car = Tgo_so * Tso_car; 
    rotationMatrix = Tgo_car.block<3, 3>(0, 0);
    euler_angles = rotationMatrix.eulerAngles( 2,1,0 );	// ZYX顺序，即yaw pitch roll顺序  取出z轴偏航角即可
    // std::cout << "原始 euler_angles(0): " << (euler_angles(0) * 180 / M_PI) << std::endl;
    // std::cout << "原始 euler_angles(1): " << (euler_angles(1) * 180 / M_PI) << std::endl;
    // std::cout << "原始 euler_angles(2): " << (euler_angles(2) * 180 / M_PI) << std::endl;

    /*****************get slam pose on map*******************************/
    euler_angles(0) = euler_angles(2)!=0 ? (M_PI - euler_angles(0)) : (2*M_PI - euler_angles(0));   //heading
    double aftYaw = (euler_angles(0) >= 0 && euler_angles(0) < M_PI/2) ?  (M_PI/2 - euler_angles(0)) : (2.5*M_PI - euler_angles(0));  //cur_pose

    // 准换后的2D位姿 
    slam_pose.x = Tgo_car(0, 3); 
    slam_pose.y = Tgo_car(1, 3); 
    slam_pose.theta = aftYaw * 180 / M_PI;  
}



int size = 0;
void onLaneLineArrayMsgRecvd(const geometry_msgs::PoseArray::ConstPtr& msg)
{
        
    // 遍历 PoseArray 中的位姿信息
    for (const auto& pose : msg->poses)
    {   
        size++;
        // double x = pose.position.x;
        // double y = pose.position.y;
        // cout<< "x = "<<x<<" "<<"y = "<<y<<endl;
        // 打印位置和方向信息
        // ROS_INFO("position: (%f, %f, %f)", pose.position.x, pose.position.y, pose.position.z);
        // ROS_INFO("orientation: (%f, %f, %f, %f)", pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    slam2gnssmap(geometry_msgs::Pose::ConstPtr(&pose, [](const geometry_msgs::Pose*){}));
     
    cout<< "x = "<<slam_pose.x<<" "<<"y = "<<slam_pose.y<<endl;
    
    // refer_points.x.push_back(slam_pose.x);
    //     refer_points.y.push_back(slam_pose.y);
    }
    cout<< "size = "<<size<<endl;
}

int main(int argc, char  *argv[])
{
    setlocale(LC_ALL,"");

    ros::init(argc,argv,"points_sub");

    ros::NodeHandle nh;



    ros::Subscriber sub_searched_points = nh.subscribe("/run_hybrid_astar/searched_points", 10, onLaneLineArrayMsgRecvd);


    

    ros::spin();//循环读取接收的数据，并调用回调函数处理

    return 0;
}
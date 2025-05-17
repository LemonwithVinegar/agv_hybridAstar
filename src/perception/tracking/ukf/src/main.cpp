#include "ukf.h"
#include <ros/ros.h>
using Eigen::MatrixXd;
using Eigen::VectorXd;

ros::Publisher Object_pub;
UKF ukf_obj[5000];
void ass_obj_callback(const custom_msgs::ObjectArray &ass_obj)
{
    double current_time = ass_obj.header.stamp.toSec();
    // std::cout << "delta_t:" << current_time-ass_obj.header.stamp.toSec()  <<std::endl; 
    // std::cout << "时间祯1:" << ass_obj.header.stamp <<std::endl;
    // std::cout << "时间祯2:" << current_time <<std::endl;
    custom_msgs::ObjectArray UKF_Objects;
    for(int i = 0; i < ass_obj.objs.size(); i++)
    {
        if(ukf_obj[ass_obj.objs[i].id].flag == 0)
        {
            ukf_obj[ass_obj.objs[i].id].flag = 1;
            // std::cout << "id:" << ass_obj.objs[i].id << std::endl;
        }
        else
        {
            custom_msgs::Object UKF_Object;
            ukf_obj[ass_obj.objs[i].id].ProcessMeasurement(ass_obj.objs[i], current_time);
            // ukf_obj[ass_obj.objs[i].id].ProcessMeasurement(ass_obj.objs[i]);
            UKF_Object.x_pos = ukf_obj[ass_obj.objs[i].id].x_[0];
            UKF_Object.y_pos = ukf_obj[ass_obj.objs[i].id].x_[1];
            UKF_Object.vx = ukf_obj[ass_obj.objs[i].id].x_[2];
            UKF_Object.vy = ukf_obj[ass_obj.objs[i].id].x_[3];
            UKF_Object.bbox_point = ass_obj.objs[i].bbox_point;
            UKF_Object.id = ass_obj.objs[i].id;
            UKF_Objects.objs.push_back(UKF_Object);
        }
    }
    Object_pub.publish(UKF_Objects);
}


int main(int argc, char** argv) 
{
    ros::init(argc, argv, "ukf_name");
    ros::NodeHandle nh;
    ros::Subscriber sub_ass_obj = nh.subscribe("/ass_object",1, ass_obj_callback);   
    Object_pub = nh.advertise<custom_msgs::ObjectArray>("/ObjectArray",1,true);  
    ros::spin();
    return 0;
}
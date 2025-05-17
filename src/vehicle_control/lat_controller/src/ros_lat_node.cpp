#include "ros_interface_lat.h"
#include "pure_pursuit.h"

//-->>全局变量
custom_msgs::Request     request_value;
// request_value = {LON_STATUS::FORWARD_ENABLE,0,0,0};

void set_requre(custom_msgs::Request &arg){
    request_value = arg;
    return ;
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pure_pursuit_node");
    ros::NodeHandle n_handle;
    //->话题订阅 ：参考路径(Path planning)、车辆状态（vehicle_chassis）
    ros::Subscriber sub_ref_path    = n_handle.subscribe("raw_path",1,ref_path_callback);
    ros::Subscriber sub_veh_statu   = n_handle.subscribe("VehicleStat",1,veh_status_callback);
    ros::Subscriber sub_Request   = n_handle.subscribe("Request",1,Request_callback);

    //->话题发布 ：方向控制命令
    ros::Publisher  pub_steer       = n_handle.advertise<geometry_msgs::Twist>("pix/lat_com_vel",1);
    //-->>设置主线程循环周期
    ros::Rate loop_rate(200);

    //->初始化车辆参数
    ros_veh_para_init();

    geometry_msgs::Twist steer_msg;
    request_value.reques_type = LON_STATUS::FORWARD_ENABLE;
    request_value.run_speed = 0;
    request_value.stop_distance = 0;
    request_value.aeb_distance = 0;
    while(ros::ok()){
      //float temp = pure_pursuit();
      if(request_value.reques_type == BACK_ENABLE)
        steer_msg.angular.z = -pure_pursuit(); //得到发布角度(弧度)
      else
        steer_msg.angular.z = pure_pursuit(); //得到发布角度(弧度)
      //std::cout << "temp = " << temp << std::endl;
      //steer_msg.SteeringAngle =300;
      ROS_INFO_THROTTLE(1,"steer_angle = %f",steer_msg.angular.z);

      //转向消息
      pub_steer.publish(steer_msg);
      ros::spinOnce();
      loop_rate.sleep();
    }
    return 0;
}

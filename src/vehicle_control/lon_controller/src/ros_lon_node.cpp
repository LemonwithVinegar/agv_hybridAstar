#include "ros_interface_lon.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "lon_controller_node");
    ros::NodeHandle n_handle;
//"pix/motor_info"
    //->话题订阅 ：车辆规划状态 + 车辆控制模式（手动/自动）
    ros::Subscriber sub_run_req     = n_handle.subscribe("Request",1,run_req_callback);
    // ros::Subscriber sub_run_mode     = n_handle.subscribe("pix/system_status",1,run_mode_callback);
    // ros::Subscriber subMotorSpeed     = n_handle.subscribe("pix/motor_info",1,motorSpeedCallback);
    //->话题发布 ：车速与转角
    ros::Publisher  pub_comvel = n_handle.advertise<geometry_msgs::Twist>("pix/lon_com_vel",1);
    
	geometry_msgs::Twist comvel;

    //-->>设置主线程循环周期 200ms
    ros::Rate loop_rate(200);
    //geometry_msgs::Twist temp_cmd;
    while(ros::ok())
    {
        ros::spinOnce();
        //请求行驶模式处理

        switch (request_value.request_type) {
            case FORWARD_ENABLE:
                    comvel = run_solve();
                    break;
            case BACK_ENABLE:
                    comvel = back_solve();
                    break;
            case STOP_ENABLE:
                    comvel = stop_solve();
                    break;
            case AEB_ENABLE:
                    comvel = stop_solve();
                    break;
        }
        //std::cout << "comvel.linear.x = " << comvel.linear.x << std::endl;
        //发布命令
        pub_comvel.publish(comvel);
        // usleep(1000*20);
        loop_rate.sleep();
    }
    return 0;
}

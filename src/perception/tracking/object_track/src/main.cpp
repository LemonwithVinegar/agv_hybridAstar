#include "ros/ros.h"
#include "data_association.h"
// #include<chrono>
// using namespace std::chrono;

 ros::Publisher pub_result;
 Data_association  association;
 custom_msgs::LaneLineArray lane_line;
//  void chatterCallback_image(const custom_msgs::ObjectArrayConstPtr& image_data)
//  {
//      association.set_raw_ImageRawObject(image_data);
//  }

void chatterCallback_lidar(const custom_msgs::LidarRawObjectArrayConstPtr& lidar_data)//custom_msgs相当于命名空间
{
    // auto start = system_clock::now();

    custom_msgs::ObjectArray msgout;//要发布出去的数据
    msgout.header.stamp = lidar_data->head.stamp;
    msgout.header.frame_id = lidar_data->head.frame_id;
    association.get_time(msgout);
    association.set_raw_LidarRawObject(lidar_data);
    // association.initial_filter();
    association.init();
    association.associate_process();
    // association.filter();
    association.filter_kalman();
    association.get_result_association(msgout,lane_line);
    msgout.header.stamp = lidar_data->head.stamp;
    msgout.header.frame_id = lidar_data->head.frame_id;
    pub_result.publish(msgout);

    // auto end = system_clock::now();
	// auto duration = duration_cast<microseconds>(end - start);
	// std::cout << "耗时："
	// 	<< double(duration.count()) * microseconds::period::num / microseconds::period::den
	// 	<< "s" << std::endl;
}
void lane_callback(const custom_msgs::LaneLineArray &lane_msg)
{
    lane_line.lines.clear();
    lane_line.lines.push_back(lane_msg.lines[0]);
}

int main(int argc,char**argv)
{
    
    ros::init(argc,argv,"data_association");//
    ros::NodeHandle n;//talking with surroundings to subscriber

    ros::Subscriber sub_lidar = n.subscribe("/detect_topic",1, chatterCallback_lidar);   //接受
     //ros::Subscriber sub_lidar=n.subscribe("/no_filter_detect_topic",1,chatterCallback_lidar);

    ros::Subscriber refer_lane_sub;
    refer_lane_sub = n.subscribe("/road_lane", 1, lane_callback); //订阅车道线
    pub_result = n.advertise<custom_msgs::ObjectArray>("/ass_object",1,true);//发布
    ros::spin();
    return 0;
}

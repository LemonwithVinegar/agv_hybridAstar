#include "ros/ros.h"
#include "association.h"
#include <ctime>
#include <compute/frenet.h>

using namespace std;

ros::Publisher pub_obj;
custom_msgs::LaneLine lane_line;
custom_msgs::ObjectArray obj_result;
association ass_core;
clock_t start_time, end_time;


typedef struct BOX_POINT{
	double x;
	double y;
} BOX_POINT;

vector<BOX_POINT> trans_angle(const double &center_x,const double &center_y,
                                const double &l,const double &w,const double &heading){
	vector<BOX_POINT> box_point;
	double angle = heading - PI/2;//heading - PI/2;

    BOX_POINT temp;
    double x,y;
    x = center_x - w/2;
    y = center_y - l/2;
    temp.x =  (x - center_x)*cos(angle) - (y-center_y)*sin(angle) + center_x;
    temp.y =  (y - center_y)*cos(angle) + (x-center_x)*sin(angle) + center_y;
    box_point.push_back(temp);

    x = center_x + w/2;
    y = center_y - l/2;
    temp.x =  (x - center_x)*cos(angle) - (y-center_y)*sin(angle) + center_x;
    temp.y =  (y - center_y)*cos(angle) + (x-center_x)*sin(angle) + center_y;
    box_point.push_back(temp);

    x = center_x + w/2;
    y = center_y + l/2;
    temp.x =  (x - center_x)*cos(angle) - (y-center_y)*sin(angle) + center_x;
    temp.y =  (y - center_y)*cos(angle) + (x-center_x)*sin(angle) + center_y;
    box_point.push_back(temp);

    x = center_x - w/2;
    y = center_y + l/2;
    temp.x =  (x - center_x)*cos(angle) - (y-center_y)*sin(angle) + center_x;
    temp.y =  (y - center_y)*cos(angle) + (x-center_x)*sin(angle) + center_y;
    box_point.push_back(temp);

	return box_point;

}

void OnlidarRecv(const custom_msgs::LidarRawObjectArrayConstPtr& msg){
    
    start_time = clock();
    ass_core.get_measure_tar_and_work(msg);
    obj_result.header.stamp = msg->head.stamp;
    obj_result.header.frame_id = msg->head.frame_id;
    obj_result.objs.clear();
    for(auto it = ass_core.targets_tracked.begin();it != ass_core.targets_tracked.end();it++){
        custom_msgs::Object temp;
        temp.id = it->id;
        if(temp.id == 0)
            continue;
        temp.type = -1;
        temp.vx = it->info_evlt.vx;
        temp.vy = it->info_evlt.vy;
        temp.v = sqrt(temp.vx*temp.vx + temp.vy*temp.vy )*3.6;
        temp.lwh.x = it->info_evlt.l;
        temp.lwh.y = it->info_evlt.w;
        temp.lwh.z = it->info_evlt.h;
        temp.x_pos = it->info_evlt.x_pos;
        temp.y_pos = it->info_evlt.y_pos;
        if(!lane_line.s.empty()){
            std::vector<double> object_s_d = getFrenet2(temp.x_pos,temp.y_pos,lane_line.x,lane_line.y,lane_line.s[0]); 
            temp.s_pos = object_s_d[0];
            temp.d_pos = object_s_d[1];
        }
        //if(it->time_match > 20){
        if(1){
            vector<BOX_POINT> temp_point;
            temp_point = trans_angle(temp.x_pos,temp.y_pos,temp.lwh.x,temp.lwh.y,it->info_evlt.heading);
            for (uint m = 0;m < 4; m++){
                temp.bbox_point[m].x = temp_point[m].x;
                temp.bbox_point[m].y = temp_point[m].y;
                temp.bbox_point[m].z = 0;
                temp.bbox_point[m+4].x = temp_point[m].x;
                temp.bbox_point[m+4].y = temp_point[m].y;
                temp.bbox_point[m+4].z = temp.lwh.z;
            }
        }else{
            for (uint m = 0;m < 4; m++){
                temp.bbox_point[m].x = it->point[m][0];
                temp.bbox_point[m].y = it->point[m][1];
                temp.bbox_point[m].z = 0;
                temp.bbox_point[m+4].x = it->point[m][0];
                temp.bbox_point[m+4].y = it->point[m][1];
                temp.bbox_point[m+4].z = temp.lwh.z;
            }
        }
        
        obj_result.objs.push_back(temp);
    }

    // custom_msgs::Object temp;
    // temp.id = 9999;
    // temp.type = -1;
    // temp.vx = 5;
    // temp.vy = 5;
    // temp.v = sqrt(temp.vx*temp.vx + temp.vy*temp.vy );
    // temp.lwh.x = 10;
    // temp.lwh.y = 5;
    // temp.lwh.z = 2;
    // temp.x_pos = 20;
    // temp.y_pos = 2;
    // if(!lane_line.s.empty()){
    //     // std::vector<double> object_s_d = getFrenet2(temp.x_pos,temp.y_pos,lane_line.x,lane_line.y,lane_line.s[0]); 
    //     // temp.s_pos = object_s_d[0];
    //     // temp.d_pos = object_s_d[1];
    // }
    
    // vector<BOX_POINT> temp_point;
    // temp_point = trans_angle(temp.x_pos,temp.y_pos,temp.lwh.x,temp.lwh.y,90);
    // for (uint m = 0;m < 4; m++){
    //     temp.bbox_point[m].x = temp_point[m].x;
    //     temp.bbox_point[m].y = temp_point[m].y;
    //     temp.bbox_point[m].z = 0;
    //     temp.bbox_point[m+4].x = temp_point[m].x;
    //     temp.bbox_point[m+4].y = temp_point[m].y;
    //     temp.bbox_point[m+4].z = temp.lwh.z;
    // }
    
    // obj_result.objs.push_back(temp);

    pub_obj.publish(obj_result);

    end_time = clock();
    //cout<<"the track_process time is : "<<(double)(end_time - start_time) / CLOCKS_PER_SEC<<"s."<<endl<<endl;

}

void OnlaneRecv(const custom_msgs::LaneLineArray &msg){
    lane_line = msg.lines[0];
}

int main(int argc,char**argv){
    ros::init(argc,argv,"obj_track");
    ros::NodeHandle n;

    //ros::Subscriber sub_lidar = n.subscribe("/no_filter_detect_topic",1, OnlidarRecv); 
    ros::Subscriber sub_lidar = n.subscribe("/detect_topic",1, OnlidarRecv); 
    ros::Subscriber sub_lane = n.subscribe("/road_lane", 1, OnlaneRecv);;
    pub_obj = n.advertise<custom_msgs::ObjectArray>("/ass_object",1,true);
    ros::spin();
    
    return 0;
}
#include <stdlib.h>
#include <mysql.h>
#include <string>
#include <iostream>
#include <vector>
#include <thread>
#include "datatype.h"
#include "read_mysql_core.h"

#include <ros/ros.h>
#include <custom_msgs/RoadAttri.h>
#include <custom_msgs/Task.h>
#include <custom_msgs/App.h>
#include <custom_msgs/CurPose.h>
using namespace std;

/**********TASK***********/


struct DBAddr db_addr = {"sde","123456","127.0.0.1","taskdb",5432};   //postgresql 
//struct DBAddr db_addr = {"taskrd","123456","localhost","taskdb",0};   //xavier_1  MySQL DB
// struct DBAddr db_addr = {"taskreader","taskRD.123","localhost","taskdata",0};   //lsxing ubuntu MySQL DB
Read_Mysql_Core read_mysql(db_addr);
ros::ServiceClient task_exc_client;
custom_msgs::CurPose curpose_msg;
struct RoadAttribute road_atr; 
struct TaskAttribute task_atr; 
custom_msgs::Task task;   
/**********TASK***********/

/**********APP***********/
ros::ServiceClient link2app_client;
custom_msgs::App app;
string appmsg_buff;
bool msg_chag = false;
ros::ServiceClient map_pub_client;
custom_msgs::Task map_chge; 
/**********APP***********/


void onCurrentPoseMsgRecvd(const custom_msgs::CurPose::ConstPtr &msg)
{
          curpose_msg = *msg;
}


void GetRoadAtr()
{
   //curpose_msg.s = 45;
   for(int i=0;i<read_mysql.road_atr_vec.size();i++)
	{
			if(read_mysql.road_atr_vec[i].start_s < curpose_msg.s && curpose_msg.s < read_mysql.road_atr_vec[i].end_s)
			   road_atr = read_mysql.road_atr_vec[i];
	 }
	 return ;
}

void GetTaskAtr(int task_execu_flag[])
{

	//curpose_msg.s = 35;
	// ROS_WARN_STREAM_THROTTLE(1,"read_mysql.task_atr_vec.size() = %d",read_mysql.task_atr_vec.size());
	for(int i=0;i<read_mysql.task_atr_vec.size();i++)
		{
			
			if(read_mysql.task_atr_vec[i].start_s < curpose_msg.s && curpose_msg.s < read_mysql.task_atr_vec[i].end_s )
			{
				if(!task_execu_flag[i]){
		
					    // ROS_WARN_STREAM_THROTTLE(10,"22222222222222222222222222222222");
                    task_atr = read_mysql.task_atr_vec[i];

					task.request.task_type = task_atr.task_type;    
					task.request.s_start = task_atr.start_s;
					task.request.s_end = task_atr.end_s;
					task.request.info = task_atr.task_info;
					//task.request.info_2 = atof(task_atr.notes);
					task.request.info_2 = 1000;
					//cout<<"after info_2: "<<task.request.info_2<<endl;

		           if( task_exc_client.call(task) ) {
		      ROS_WARN_STREAM_THROTTLE(10,"Task srv called____________________");
                     if(task.response.isSuccess == true)
				         task_execu_flag[i] = true;
			         else 
                         ROS_ERROR_STREAM_THROTTLE(1,"task_exc_service's response is failed");	 
		            }
                   else ROS_ERROR_STREAM_THROTTLE(1,"Failed	to	call  service	task_exc_service");
 
		      }	
			  else ;

		  }

		  else task_execu_flag[i] = false;
		}

	 return ;
}

void Exe_appmsg_func()  //ros::ServiceClient link2app_client
{
	ros::Rate loop_rate(10);
    // cout<<"i am Exe_appmsg_func"<<endl;
	int recv_num = 0;
	app.request.isSend = 0;
    app.request.sendbuf = "I am receper";

	while(ros::ok()){
	loop_rate.sleep();
	
    if (link2app_client.call(app)){
        //ROS_WARN_STREAM_THROTTLE(1,"!!!!!!!!!! : "<<app.response.recvbuf);
		if(app.response.recvbuf != appmsg_buff){// || msg_chag){
            
            appmsg_buff = app.response.recvbuf;
			msg_chag = true;
			recv_num++;

			if( appmsg_buff == "GoOrigin" ){//&& msg_chag && recv_num > 0) {   //Is_change_map

			map_chge.request.task_type = 6;    
			map_chge.request.s_start = 0;
			map_chge.request.s_end = 0;
			map_chge.request.info = 1;
			map_chge.request.info_2 = 0;

		    if( map_pub_client.call(map_chge) ) {
		    // ROS_WARN_STREAM_THROTTLE(10,"Task srv called____________________");
                while(map_chge.response.isSuccess == false){
					ROS_ERROR_STREAM_THROTTLE(1,"task_exc_service's response is failed");
					ros::Duration(2).sleep();
					map_pub_client.call(map_chge);
				}

			   recv_num = 0;
			   msg_chag = false;

			}
			else ROS_ERROR_STREAM_THROTTLE(1,"task_exc_service's response is failed");

	}


	if( appmsg_buff == "GetOrigi" ){//&& msg_chag && recv_num > 0) {   //Is_change_map

			map_chge.request.task_type = 6;    
			map_chge.request.s_start = 0;
			map_chge.request.s_end = 0;
			map_chge.request.info = 3;
			map_chge.request.info_2 = 0;
            
		    if( map_pub_client.call(map_chge) ) {
		    // ROS_WARN_STREAM_THROTTLE(10,"Task srv called____________________");
                while(map_chge.response.isSuccess == false){
					ROS_ERROR_STREAM_THROTTLE(1,"task_exc_service's response is failed");
					ros::Duration(2).sleep();
					map_pub_client.call(map_chge);
				}

			   recv_num = 0;
			   msg_chag = false;

			}
			else ROS_ERROR_STREAM_THROTTLE(1,"task_exc_service's response is failed");

	}


	if( appmsg_buff == "GoDes" ){//&& msg_chag && recv_num > 0) {   //Is_change_map

			map_chge.request.task_type = 6;    
			map_chge.request.s_start = 0;
			map_chge.request.s_end = 0;
			map_chge.request.info = 5;
			map_chge.request.info_2 = 0;

		    if( map_pub_client.call(map_chge) ) {
		     ROS_WARN_STREAM_THROTTLE(10,"Task srv called____________________");
                while(map_chge.response.isSuccess == false){
					ROS_ERROR_STREAM_THROTTLE(1,"task_exc_service's response is failed");
					ros::Duration(2).sleep();
					map_pub_client.call(map_chge);
				}

			   recv_num = 0;
			   msg_chag = false;

			}
			else ROS_ERROR_STREAM_THROTTLE(1,"task_exc_service's response is failed");

	}



	// if( appmsg_buff == "GetDes" ){//&& msg_chag && recv_num > 0) {   //Is_change_map

	// 		map_chge.request.task_type = 6;    
	// 		map_chge.request.s_start = 0;
	// 		map_chge.request.s_end = 0;
	// 		map_chge.request.info = 7;
	// 		map_chge.request.info_2 = 0;

	// 	    if( map_pub_client.call(map_chge) ) {
	// 	    // ROS_WARN_STREAM_THROTTLE(10,"Task srv called____________________");
    //             while(map_chge.response.isSuccess == false){
	// 				ROS_ERROR("task_exc_service's response is failed");
	// 				ros::Duration(2).sleep();
	// 				map_pub_client.call(map_chge);
	// 			}

	// 		   recv_num = 0;
	// 		   msg_chag = false;

	// 		}
	// 		else ROS_ERROR("task_exc_service's response is failed");

	// }


	if( appmsg_buff == "BackInit" ){//&& msg_chag && recv_num > 0) {   //Is_change_map

			map_chge.request.task_type = 6;    
			map_chge.request.s_start = 0;
			map_chge.request.s_end = 0;
			map_chge.request.info = 9;
			map_chge.request.info_2 = 0;

		    if( map_pub_client.call(map_chge) ) {
		    // ROS_WARN_STREAM_THROTTLE(10,"Task srv called____________________");
                while(map_chge.response.isSuccess == false){
					ROS_ERROR_STREAM_THROTTLE(1,"task_exc_service's response is failed");
					ros::Duration(2).sleep();
					map_pub_client.call(map_chge);
				}

			   recv_num = 0;
			   msg_chag = false;

			}
			else ROS_ERROR_STREAM_THROTTLE(1,"task_exc_service's response is failed");

	}

		}
        
        //cout << appmsg_buff << endl;
    }
    else{
        //ROS_ERROR_STREAM_THROTTLE(1,"task_pub_node : Failed to call service app");
    }

	}


}







int main(int argc, char **argv)
{
	
    ros::init(argc, argv, "task_pub_node");
    ros::NodeHandle nh;
    
	//custom_msgs::Task srv;
	custom_msgs::RoadAttri road_attri_msg;

	ros::Publisher road_attri_pub = nh.advertise<custom_msgs::RoadAttri>("/road_attri_msg", 1);
	ros::Subscriber sub_cur_pose = nh.subscribe("/cur_pose_all", 1 ,onCurrentPoseMsgRecvd);
	task_exc_client  = nh.serviceClient<custom_msgs::Task>("/task_exc_service");
	link2app_client  = nh.serviceClient<custom_msgs::App>("/link2app_service");
	map_pub_client  = nh.serviceClient<custom_msgs::Task>("/task_exc_service");
 

    ros::Timer timer = nh.createTimer(ros::Duration(0.1),
                                      [&road_attri_msg,&road_attri_pub]  //,&road_atr
                                      (const ros::TimerEvent& evnt){

		road_attri_msg.velocity = road_atr.velocity;
		road_attri_msg.road_width = road_atr.road_width;

		road_attri_msg.aeb_front = road_atr.aeb_front;
		road_attri_msg.aeb_back = road_atr.aeb_back;
		road_attri_msg.aeb_left = road_atr.aeb_left;
		road_attri_msg.aeb_right = road_atr.aeb_right;

		road_attri_msg.detect_front = road_atr.detect_front;
		road_attri_msg.detect_back = road_atr.detect_back;
		road_attri_msg.detect_left = road_atr.detect_left;
		road_attri_msg.detect_right = road_atr.detect_right;
		
		road_attri_msg.detect_right = road_atr.detect_right;
        road_attri_pub.publish(road_attri_msg);

    });

    thread Exe_appmsg_th( Exe_appmsg_func );  //link2app_client
	Exe_appmsg_th.detach();
	
	ros::Rate loop_rate(10);

    //cout<<"!!!!!!!!!!!!!!"<<endl;
	read_mysql.Read_Pqdata();
	while( !read_mysql.connect_is_ok ){
		read_mysql.Read_Pqdata();
	}
	int task_execu_flag[read_mysql.task_atr_vec.size()] = {0};
        //cout<<"!!!!!!!!!!!!!!"<<endl;  


	// read_mysql.MysqlInit();
	// read_mysql.Read_Road_Attri();   
	// read_mysql.Read_Task_Attri();
	// if(!read_mysql.connect_is_ok) 
	// 	ROS_WARN_STREAM_THROTTLE(10,"connect database failed!!!!!!!!!!!!!!!!!!!!!");
	// if(!read_mysql.query_is_ok) 
	// 	ROS_WARN_STREAM_THROTTLE(10,"query sql failed!!!!!!!!!!!!!!!!!!!!!");
    // read_mysql.Free_Mysql();

	while(ros::ok()) {
		
		ros::spinOnce();

		GetRoadAtr();

                GetTaskAtr(task_execu_flag);

    	loop_rate.sleep();
		
    }

	return 0;
 
}

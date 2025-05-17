#include "hybrid_astar_task.h"







void HybridAstarTask::onSegmentMsgRecvd(const custom_msgs::CarDirection::ConstPtr &msg)
{
    segment_msg = *msg;
}


void HybridAstarTask::onCurrentPoseMsgRecvd(const custom_msgs::CurPose::ConstPtr &msg)
{
    curpose_msg = *msg;
	
}

void HybridAstarTask::timerCallback(const ros::TimerEvent& evnt) {
	custom_msgs::RoadAttri road_attri_msg;
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
	// cout << "velocity: " << road_attri_msg.velocity<<endl; 
	road_attri_pub.publish(road_attri_msg);
}

//*******************************************************加锁访问**************************************************************
/**
 * @brief 为避免异常加锁访问定位
 * @param 
 */
inline custom_msgs::CarDirection HybridAstarTask::get_CarDirection_WithMutex() {
	custom_msgs::CarDirection res;
    mtx_car_direction.lock();
    res = segment_msg;
    mtx_car_direction.unlock();
    return res;
}
//*******************************************************加锁访问**************************************************************

void HybridAstarTask::getTaskS(custom_msgs::CarDirection msg) {
		task_atr.end_s = msg.end_s;
		task_atr.start_s = task_atr.start_s;
		task_atr.task_info = task_atr.end_s;
}


vector<RoadAttribute> HybridAstarTask::readDataFromFile(const string& filename)
{
    ifstream file(filename); // 打开文件
    vector<RoadAttribute> road_atr_vec;
    
    if (file.is_open())
    {
        string line;
        getline(file, line); // 跳过第一行

        while (getline(file, line)) // 逐行读取文件
        {
            stringstream ss(line);
            float v, w, f, b, l, r, df, db, dl, dr, s, e;
            if (ss >> v >> w >> f >> b >> l >> r >> df >> db >> dl >> dr >> s >> e) // 分离每行数据
            {
                RoadAttribute road_atr;
                road_atr.velocity = v;
                road_atr.road_width = w;
                road_atr.aeb_front = f;
                road_atr.aeb_back = b;
                road_atr.aeb_left = l; 
                road_atr.aeb_right = r;
                road_atr.start_s = s;
                road_atr.end_s = e;
                road_atr_vec.push_back(road_atr);
            }
            else
            {
                cout << "Failed to parse line: " << line << endl;
            }
        }

        // 处理完数据后关闭文件
        file.close();
    }
    else
    {
        cout << "Failed to open file: " << filename << endl;
    }
    
    return road_atr_vec;
}

void HybridAstarTask::GetRoadAtr()
{
	string filepath = ros::package::getPath("hyastar_task_pub") + "/config/road_attributes.txt";
	vector<RoadAttribute> road_atr_vec = readDataFromFile(filepath);

	for (auto& roadAtr : road_atr_vec)
	{
		road_atr = roadAtr;
		// cout << "velocity: " << road_atr.velocity 
		// 		<< ", width: " << road_atr.road_width 
		// 		<< ", front: " << road_atr.aeb_front 
		// 		<< ", back: " << road_atr.aeb_back 
		// 		<< ", left: " << road_atr.aeb_left << ", right: " << road_atr.aeb_right 
		// 		<< ", start_s: " << road_atr.start_s << ", end_s: " << road_atr.end_s << endl;
	}
	return ;
}

void HybridAstarTask::hyAstarBackCarTask()
{	//前进倒车任务
	custom_msgs::CarDirection car_pose;
	car_pose = get_CarDirection_WithMutex();
		getTaskS(car_pose);
		if (car_pose.lastSegment == 0) {	//规划路径不只一段或者不是最后一段规划的路径
			// if(task_atr.start_s < curpose_msg.s && curpose_msg.s < task_atr.end_s && car_pose.path_id != pre_task_id) {
			if(curpose_msg.s < task_atr.end_s && car_pose.path_id != pre_task_id) {
				task.request.task_type = 4;   //
				//主要使用 s_start、s_end 
				task.request.s_start = task_atr.start_s;
				task.request.s_end = task_atr.end_s;
				task.request.info = task_atr.start_s;//开始s
				task.request.info_2 = task_atr.end_s;//结束s
				ros::Duration(1).sleep();
				if( task_exc_client.call(task) ) {
					ROS_WARN_STREAM_THROTTLE(10,"Task srv called____________________");
					pre_task_id = car_pose.path_id;
					if(task.response.isSuccess == true) {
						ROS_ERROR_STREAM_THROTTLE(1,"task_exc_service's response is success");
					}  
					else
						ROS_ERROR_STREAM_THROTTLE(1,"task_exc_service's response is failed"); 	 
				}
				// else ROS_ERROR_STREAM_THROTTLE(1,"Failed	to	call  service	BackCarTask task_exc_service");
			}
		}
		
	return ;
}

void HybridAstarTask::hyAstarStopCarTask() {  
	custom_msgs::CarDirection car_pose;
	car_pose = get_CarDirection_WithMutex();
	if (car_pose.lastSegment == 1) {	//规划出的路径只有一段或者路径是最后一段，才进行终点停车
		// if (car_pose.end_s != 0 && (car_pose.end_s - curpose_msg.s < 4)&& car_pose.path_id != pre_stop_id) {
		if(car_pose.end_s != 0 && car_pose.path_id != pre_stop_id) {
			getTaskS(car_pose);
			task.request.task_type = 8;    
			task.request.s_start = task_atr.start_s;
			task.request.s_end = task_atr.end_s;
			task.request.info = task_atr.task_info;
		
			if( task_exc_client.call(task) ) {
				ROS_WARN_STREAM_THROTTLE(10,"Task srv called__StopCar__________________");
				pre_stop_id = car_pose.path_id;
				
				if(task.response.isSuccess == true) {
					ROS_ERROR_STREAM_THROTTLE(1,"task_exc_service's response is success");
				}
				else 
					ROS_ERROR_STREAM_THROTTLE(1,"task_exc_service's response is failed");	 
			}
		}
		// else ROS_ERROR_STREAM_THROTTLE(1,"Failed	to	call  service	StopCarTask task_exc_service");
	}
	return;
}
 



int main(int argc, char **argv)
{
	
    ros::init(argc, argv, "hyastar_task_pub_node");

    
	HybridAstarTask task;
	ros::Rate loop_rate(10);
	while(ros::ok()) {
		
		task.GetRoadAtr();
		task.hyAstarBackCarTask();
		task.hyAstarStopCarTask();
		ros::spinOnce();
    	loop_rate.sleep();
		
    }

	return 0;
 
}


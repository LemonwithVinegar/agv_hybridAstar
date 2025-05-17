#ifndef DATA_ASSOCIATION_H
#define DATA_ASSOCIATION_H

#include "custom_msgs/LidarRawObjectArray.h"
#include "custom_msgs/LidarRawObject.h"
#include "custom_msgs/ObjectArray.h"
#include "custom_msgs/Object.h"
#include "custom_msgs/LaneLineArray.h"
#include "custom_msgs/LaneLine.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <iostream>
#include <iomanip>
#include <vector>
#include <list>
#include <string.h>
#include <fstream>
#include<compute/frenet.h>
using namespace Eigen;
//单个雷达目标信息
struct target
{
     double vx;         //目标x方向速度，m/s
     double vy;         //目标y方向速度,m/s
     double v;          //目标速度，km/h
     float wide;        //目标宽
     float deep;        //目标深度
     float height;      //目标高
     float x_pos;       //目标x坐标
     float y_pos;       //目标y坐标
     float s_pos;       //目标s坐标
     float d_pos;       //目标d坐标
     int tar_associated;          //关联flag,为1表示观测数据和预测数据被匹配,同时用于判定目标的消失与出现,出现为1
     double points[8][3];         //目标的八个顶点

     target();
     ~target();
};

//单个目标的所有信息，包含预测值和历史数据
class target_pre_his
{
public:
    uint id;                            //目标id
    int flag_new_target;                //设置为新目标，目标10次中出现7次才将其置位为1，之后一直保持1的状态
    std::vector<target>predict;         //预测数据
    std::vector<target>history;         //历史数据

    MatrixXd A = MatrixXd::Constant(4, 4, 0);   //状态转移矩阵
    MatrixXd B = MatrixXd::Constant(4, 1, 0);   //控制量
    MatrixXd H = MatrixXd::Constant(2, 4, 0);   //观测矩阵
    MatrixXd Q = MatrixXd::Constant(4, 4, 0);   //预测过程噪声协方差
    MatrixXd R = MatrixXd::Constant(2, 2, 0);   //测量过程噪声协方差
    //X_evlt:后验估计状态值,X_pdct:先验预测状态值,Z_meas:当前量测,
    //Pk:估计状态和真实状态的协方差矩阵(更新),Pk_p:预测状态与真实状态的协方差矩阵,K:滤波增益
    MatrixXd X_evlt = MatrixXd::Constant(4, 1, 0), X_pdct = MatrixXd::Constant(4, 1, 0), Z_meas = MatrixXd::Constant(2, 1, 0),
    Pk = MatrixXd::Constant(4, 4, 0), Pk_p = MatrixXd::Constant(4, 4, 0), K = MatrixXd::Constant(4, 2, 0);
    std::vector<Eigen::MatrixXd> x_evlt, x_pdct, z_meas, pk, pk_p, k;
public:
    target_pre_his();
    ~target_pre_his();
};


class Data_association
{
private:
    uint tarnum;            //用作id++
    int first_flag;         //防止第一帧进入预测关联等过程
    uint frame_count;       //帧数计数

    double delta_time;      //按实际帧间间隔取的时间
    double current_time;
    double last_time;
    double mult_time;
    std::list<double> time_save;        //存储30帧时间,每次时间更新之后栈尾入栈,栈首出栈

    custom_msgs::LidarRawObjectArray curruentlidararraydata;//雷达数据
    // custom_msgs::ImageObjectArray curruentimagearraydata;//图像数据
    std::vector<target>Z_k;
    std::vector<target_pre_his> result;//保存滤波后的结果
    // std::vector<target_pre_his> observed_10;

    std::ofstream fout2;
    std::ofstream fout1;

public:
    Data_association();
    ~Data_association();
    void set_raw_LidarRawObject(const custom_msgs::LidarRawObjectArrayConstPtr& msg);
    // void set_raw_ImageRawObject(const custom_msgs::ImageObjectArrayConstPtr& msg);
    // void initial_filter();          
    void init();                    //第一帧
    void predict();                 //目标预测
    void associate_process();       //关联处理
    void filter();                  //滤波,采用最小二乘法
    void filter_kalman();           //滤波，卡尔曼
    void get_result_association(custom_msgs::ObjectArray &msg,custom_msgs::LaneLineArray lane_line);        //传出处理结果

    double save_time(double delta_time,uint frames);                        //保存时间
    void get_time(custom_msgs::ObjectArray &msgout);
    void disppear_tar_judge(std::vector <target_pre_his> &result);          //消失和新目标的判定
    void kalman_init(target_pre_his &target_kalman);
};

#endif // DATA_ASSOCIATION_H

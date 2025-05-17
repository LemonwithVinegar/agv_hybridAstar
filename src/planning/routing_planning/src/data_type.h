#ifndef DATA_TYPE_H
#define DATA_TYPE_H

#include <vector>
#include <string>
#include <stdint.h>

using namespace std;


const float speed_proportion = 0.3;       //速度离散比例系数
//const float trans_para_forward = 0;     //将车辆定位转换到车头
const float trans_para_back = 0.5;     //将车辆定位转换到houlun 5 4
const float trans_para_park = 0;      //将车辆定位转换到车尾    2



//主车当前位姿
struct CurrentPose
{
    double x;
    double y;  //地图坐标系上
    double s;
    double d;
    double yaw;  //当前主车朝向与地图坐标系的夹角，逆时针旋转，0~360度
};

//主车当前状态
struct CurrentState
{
    float velocity;  //主车速度，单位km/h
};


//车道线信息,地图坐标系上
struct Point
{
    double x;
    double y;
    double s;
};
struct LaneLine
{
    vector<Point> points;
    int line_nums;
};

//变道信息
struct ChangeLane
{
    bool is_change_lane;                 //变道标志位
    float offset_postion;                //横向偏移目标位置
    float first_s;                       //
};

//变道
struct Backcar_task
{
  bool  enable;          //使能
};

//速度
struct Speed_task
{
  bool  enable;           //使能
  float info;             //速度
};

//AEB/STOP
struct AEB_STOP
{
  bool  enable;           //使能
  float info;             //前方停止距离
};

#endif // DATA_TYPE_H

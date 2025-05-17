#ifndef MSG_H
#define MSG_H
#include <stdint.h>

#pragma pack(1)
#define MSG          5000
//-->车辆状态----------------------------------------
typedef struct VichleStatu{
    uint16_t msg=5001;                  //消息头 MSG+1
    double   lat=102;                       //巡逻车gps经度
    double   lon=168;                       //巡逻车gps纬度
    double   speed=12;                     //巡逻车速度
    double   direct=68;                    //巡逻车航向
    double   battery_left=85;              //巡逻车剩余电量
    uint8_t  route_code=0x00;                //巡逻车线路码
    uint8_t  exception_code=0x00;            //巡逻车异常码 正常：0x00雷达异常：0x01 定位异常：0x02 车辆控制异常：0x04 控制器异常：0x08 (若有多个异常，按位相加）
    uint8_t  alert_code=0x00;                //告警代码 正常：0x00 道路异常（AEB状态持续T秒）：0x01     电量低：0x02
    uint8_t  vichle_status_code=0x00;        //巡逻车行驶状态代码 启动：0x00 前进：0x01 倒车：0x02 驻车：0x03
    uint8_t  vichle_task_code=0x00;          //巡逻车任务码 空闲：0x00 执行：0x01
    long     timestamp;                 //时间戳（通用毫秒数）
} VichleStatu;

//-->状态控制----------------------------------------
typedef struct State_Control{
    uint16_t    msg;              	    //消息头 MSG+2
    uint8_t     vichle_control_code=0x05;    //巡逻车状态控制代码  停止：0x00 暂停：0x01 恢复：0x02
} State_Control;

//-->任务线路派发-------------------------------------
typedef struct Task_Send{
    uint16_t      msg;              	//消息头 MSG+3
    uint8_t       route_code;           //巡逻车线路代码 
} Task_Send;

//-->巡逻车返回任务应答-------------------------------------
typedef struct Task_Res{
    uint16_t      msg=5004;				//消息头 MSG+4
    uint8_t       route_code;           //巡逻车线路代码
    uint8_t       status=0x04;               //任务应答状态 	任务执行中拒绝：0x00    任务线路冲突 0x01    空闲接受：0x02
} Task_Res;

#pragma pack()
#endif // MSG_H

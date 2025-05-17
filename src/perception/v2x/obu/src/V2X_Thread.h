#ifndef	_V2X_H_
#define	_V2X_H_

#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <wait.h>
#include <errno.h>
#include <netdb.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <sys/time.h>
#include <stdint.h>

/**select定时**/
#include <sys/select.h>
/**Json**/
#include "cJSON.h"
/**描述符控制**/
#include <fcntl.h>
#include <signal.h>


/****宏定义****/
#define BUFF_MAX_SIZE           1024    //字符串最大长度

#define JSON_MSGID_OBU			2101    //OBU本车信息上报
#define JSON_MSGID_SPAT			2104    //OBU红绿灯倒数秒数上报
#define JSON_MSGID_RSI			2105    //OBU事件信息上报
#define JSON_MSGID_RSI_EV		2001    //OBU事件信息上报(紧急车辆)
#define JSON_MSGID_RSI_TP		2002    //OBU事件信息上报(打车行人)


#define OBU_DEVICE_ID_SIZE      32      //OBU ID长度
#define RSU_DEVICE_ID_SIZE      32      //RSU ID长度
#define VEHICLE_NUM_SIZE        16      //车牌号长度
#define PARTICIPANT_NUM         16      //车牌号长度


#define TRUE                    1       //means buffer enable
#define FALSE                   0       //means buffer disable

#define MYPORT                  6666
/**结构体声明**/

// L2A本地模块端口号和socketfd管理结构
typedef struct SocketInfo
{
	char* pSIP;  		    //HMI服务器IP
	int SPort;               //HMI服务器端口

	struct
	{
		int OBUFD;			//HV_OBU消息
	} clientfd;
	struct sockaddr_in serverAddr;
} tAndroidSocketInfo;

//V2X模块线程周期
typedef struct V2XParams
{
	struct
	{
		uint32_t V2X;           //V2X_Thread线程
		uint32_t UDP_Listening;        //Listen线程
	} Interval;

} tV2XParams;

//HMI模块配置信息
typedef struct V2X
{
	struct V2XParams Params;        //HMI模块 线程周期
	struct SocketInfo ASI;   //Socket相关
} tV2X;



//OBU本车消息
typedef struct OBU
{
	char device_id[OBU_DEVICE_ID_SIZE];         //OBU ID
	uint8_t drive_status;                   //根据文档，为保留字段，暂不处理
	float ele;                              //根据文档，为保留字段，暂不处理
	float hea;                              //根据文档，为保留字段，暂不处理
	double lat;                             //根据文档，为保留字段，暂不处理
	double lon;                             //根据文档，为保留字段，暂不处理
	float spd;                              //根据文档，为保留字段，暂不处理
	char vehicle_num[VEHICLE_NUM_SIZE];     //根据文档，为保留字段，暂不处理
	uint8_t vehicle_type;
} tOBU;



//相位
typedef struct Phase
{
    uint8_t color;              //相位颜色
    uint8_t direction;          //当前灯色
    float guide_speed_max;      //根据文档，为保留字段，暂不处理
    float guide_speed_min;      //根据文档，为保留字段，暂不处理
    uint8_t id;                 //相位ID
    uint8_t time;               //剩余时间
} tPhase;
//SPAT红绿灯消息
typedef struct SPAT
{
    char device_id[RSU_DEVICE_ID_SIZE]; //RSU ID
    double lat;                         //红绿灯所在路口中心点的纬度
    double lon;                         //红绿灯所在路口中心点的经度
    struct Phase phase;                 //感兴趣相位
    float ele;                          //根据文档，为保留字段，暂不处理
    float hea;                          //根据文档，为保留字段，暂不处理
} tSPAT;

typedef struct BUFFER
{
    char recv_buf[BUFF_MAX_SIZE];
    int flag;


}myBUFFER;



//行人
typedef struct Participant
{
    char device_id[OBU_DEVICE_ID_SIZE]; //OBU ID
    float hea;                          //方向
    double lat;                         //车辆或事件的纬度
    double lon;                         //车辆或事件的经度
    float spd;                          //速度
    char device_type;                   //根据文档，为保留字段，暂不处理
    uint8_t direction;                  //根据文档，为保留字段，暂不处理
    float dist2crash;                   //根据文档，为保留字段，暂不处理
    float ele;                          //根据文档，为保留字段，暂不处理
    uint8_t facility_type;              //根据文档，为保留字段，暂不处理
    float ttc;                          //根据文档，为保留字段，暂不处理
    char vehicle_num[VEHICLE_NUM_SIZE]; //根据文档，为保留字段，暂不处理
    uint8_t vehicle_type;               //根据文档，为保留字段，暂不处理
} tParticipant;
//RSI交通事件消息
typedef struct RSI
{
    uint8_t type;               //事件类型：紧急车辆 2001；打车行人 2002
    float hea;                  //车辆方向
    double lat;                 //OBU 经度
    double lon;                 //OBU 纬度
    float spd;                  //OBU 速度
    struct Participant participant[PARTICIPANT_NUM];//行人
    uint8_t alarm;              //根据文档，为保留字段，暂不处理
    float ele;                  //根据文档，为保留字段，暂不处理
    uint8_t highlight;          //根据文档，为保留字段，暂不处理
    float reserved1;            //根据文档，为保留字段，暂不处理
    float reserved2;            //根据文档，为保留字段，暂不处理
    float reserved3;            //根据文档，为保留字段，暂不处理
    uint8_t time_display;       //根据文档，为保留字段，暂不处理
    char toast[32];             //根据文档，为保留字段，暂不处理
    char voice[32];             //根据文档，为保留字段，暂不处理
} tRSI;

struct obu_msg
{
	int status;
	float lefttime;
};


/**外部函数声明**/
extern void* V2X_Thread ( void* arg );
extern void* Obu_Thread ( void* arg );
extern void UtilNap ( uint32_t us );
extern int Data_analysis(float direction, double *color, double *time, int *IsPickUp, int *IsAmbulance);
extern struct obu_msg msg;

#endif









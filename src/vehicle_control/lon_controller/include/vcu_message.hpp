#pragma once
/***********************************************************************/
/* 说明：因为需要UdpSystemStatus_t类型来接收AGV底盘控制节点（pix_robot_can_node）传过来的系统 手动/自动控制模式标志而从pix_robot_can移植过来的
/***********************************************************************/
#include <stdint.h>
#include <stdbool.h>

// #define LCD_MAX_CHAR_NUM 32

// enum UDP_MSG_TYPE {
// 	UDP_MSG_TYP_HEARTBEAT = 12000,
// 	UDP_MSG_TYP_SET_MOTOR_SPEED,
// 	UDP_MSG_TYP_SET_MOTOR_PARAMETERS,
// 	UDP_MSG_TYP_CONTROL_PARKING_BRAKE,
// 	UDP_MSG_TYP_CONTROL_CARGO_UNLOCK,
// 	UDP_MSG_TYP_CONTROL_LIGHTS,
// 	UDP_MSG_TYP_REPORT_MOTOR_INFO,
// 	UDP_MSG_TYP_REPORT_SONAR_DATA,
// 	UDP_MSG_TYP_REPORT_SYS_STATUS,
// 	UDP_MSG_TYP_DISPLAY_ON_LCD,
// 	UDP_MSG_TYP_RESET_MOTOR,
// 	UDP_MSG_TYP_IMPACT_PROTECT,
// 	UDP_MSG_TYP_CONTROL_CARGO_LID,
// 	UDP_MSG_TYPE_SET_ROBOT_MAX_SPEED,
// 	UDP_MSG_TYPE_SET_ROBOT_MOTION,
// 	UDP_MSG_TYPE_MOTOR_CONTROL_FUNCTION,
// 	UDP_MSG_TYPE_SET_EMERGENCY,
// 	UDP_MSG_TYPE_SET_ACKERMANN_MOTION,
// 	UDP_MSG_TYPE_SET_STEERING_MODE
// };

// enum MOTOR_CONTROL_ACTION_TYPE {
// 	MOTOR_CONTROL_ACTION_RELEASE = 0x300,
// 	MOTOR_CONTROL_ACTION_START,
// 	MOTOR_CONTROL_ACTION_RESET
// };


/* --------- UDP data reported from VCU --------- */

// Sonar data
// #define SONAR_TOTAL_NUM	8
// typedef struct
// {
// 	uint32_t msgTyp;
// 	uint16_t distance[SONAR_TOTAL_NUM];
// } UdpSonarData_t;

// // Motor information
// #define MOTOR_TOTAL_NUM	4
// typedef struct
// {
// 	uint32_t msgTyp;
// 	uint32_t runningStatus[MOTOR_TOTAL_NUM];	// 0-Normal 1-Over volatage 2-Over current 3-Over load 4-Hall sensor error 5-Current overshoot 6-Encoder overshoot 7-Speed overshoot 8-Reference volate error
// 	int32_t speed[MOTOR_TOTAL_NUM];				// current running speed in 0.1RPM
// 	int32_t position[MOTOR_TOTAL_NUM];			// encouder counts
// 	float temperature[MOTOR_TOTAL_NUM];			// motor's temperature.
// } UdpMotorInfo_t;


// System status
typedef struct
{
	uint32_t msgTyp;
	uint32_t upTime;			// Power on time
	uint32_t errorCode;			// 0 - Normal 1 - Motor fault 2 - Low Voltage 3 - Overheating 4 - VCU no response
	uint16_t brakeStatus;		// Brake
	int32_t temperature;		// Temperature in box
	uint16_t cargoLock;			// Cargo lock status
	uint16_t cargoLidClosed;	// If the cargo lid is closed
	float voltage;				// Battery voltage
	float current;				// Total currency
	uint32_t capacity;			// Battery remain capacity in percentage
	uint16_t inCharge;			// If in charge or not
	uint16_t frontEmStop;		// Front emergency stop
	uint16_t rearEmStop;		// Rear emergency stop
	uint16_t remoteEmStop;		// Remote controller emergency stop
	uint16_t frontRearImpacted;	// If front or rear sensors detected impact
	uint16_t sideImpacted;		// If side sensors detected impact
	uint32_t impactBitmap;		// Bitmap which shows which sensor/sensors caused the impact.
	uint16_t manualMode;		// If the robot controlled manually.
	uint16_t rosEmStop;			// ROS emergency stop.
} UdpSystemStatus_t;

// /* -------- Send UDP data to VCU ---------------- */
// #define MOTOR_TOTAL_NUM	4
// typedef struct
// {
// 	uint32_t msgType;
// } UdpHeartbeat_t;

// typedef struct
// {
//     uint32_t msgType;
//     uint32_t motorSpeed[MOTOR_TOTAL_NUM];
// } UdpSetMotorSpeed_t;

// typedef struct
// {
//     uint32_t msgType;
//     int32_t turnSignal;
//     uint32_t chassisLight;
//     uint32_t headLight;
//     uint32_t flagLight;
// } UdpLightControl_t;

// typedef struct
// {
//     uint32_t msgType;
//     uint32_t distance;
// } UdpMoveStraight_t;

// typedef struct
// {
//     uint32_t msgType;
//     uint32_t degree;
// } UdpSpotTurn_t;

// typedef struct
// {
//     uint32_t msgType;
//     uint32_t maxSpeed;
//     uint32_t accTime;
//     uint32_t decTime;
// } UdpSetMotorParameters_t;

// typedef struct
// {
// 	uint32_t msgType;
// 	uint32_t ifBrake;
// } UdpControlParkingBrake_t;

// typedef struct
// {
// 	uint32_t msgType;
// 	uint32_t ifUnlock;
// } UdpControlCargoUnlock_t;

// typedef struct
// {
// 	uint32_t msgType;
// 	int32_t turnSignal;
// 	uint32_t chassisLights;
// 	uint32_t headLights;
// 	uint32_t flagLight;
// } UdpControlLight_t;

// typedef struct
// {
//     uint32_t msgType;
//     uint8_t firstLine[LCD_MAX_CHAR_NUM/2];
// 	uint8_t secondLine[LCD_MAX_CHAR_NUM/2];
// } UdpDisplayOnLcd_t;

// typedef struct
// {
// 	const uint32_t msgType = UDP_MSG_TYP_RESET_MOTOR;
// 	int32_t motorId;
// } UdpResetMotor_t;

// typedef struct
// {
// 	const uint32_t msgType = UDP_MSG_TYP_IMPACT_PROTECT;
// 	uint16_t enableFrontRear;
// 	uint16_t enableSide;
// } UdpImpactProtect_t;

// typedef struct
// {
// 	uint32_t msgType = UDP_MSG_TYP_CONTROL_CARGO_LID;
// 	uint32_t action;	// 0 : No action; 1 : Close; 2 : Open;
// } UdpOpenCargoLid_t;

// typedef struct {
// 	uint32_t msgType = UDP_MSG_TYPE_SET_ROBOT_MAX_SPEED;
// 	uint32_t maxSpeed;
// } UdpSetRobotMaxSpeed_t;

// typedef struct {
// 	uint32_t msgType = UDP_MSG_TYPE_SET_ROBOT_MOTION;
// 	float velocity;
// 	float angularVelocity;
// } UdpSetRobotMotion_t;

// typedef struct {
// 	uint32_t msgType = UDP_MSG_TYPE_MOTOR_CONTROL_FUNCTION;
// 	uint16_t motorId;
// 	uint16_t action;
// } UdpMotorControlFunction_t;

// typedef struct {
// 	uint32_t msgType = UDP_MSG_TYPE_SET_EMERGENCY;
// 	uint16_t emergencyStatus;
// } UdpSetEmergency_t;

// typedef struct {
// 	uint32_t msgType = UDP_MSG_TYPE_SET_ACKERMANN_MOTION;
// 	float velocity;
// 	float ackermannAngle;
// } UdpSetAckermannAngle_t;

// typedef struct {
// 	uint32_t msgType = UDP_MSG_TYPE_SET_STEERING_MODE;
// 	uint32_t steeringMode;
// } UdpSetSteeringMode_t;








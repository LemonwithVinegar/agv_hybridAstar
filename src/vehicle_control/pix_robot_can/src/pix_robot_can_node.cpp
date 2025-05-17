#include <thread>
#include <signal.h>

#include <ros/ros.h>

#include "can_interface.hpp"
#include "udp_interface.hpp"
#include "message_callbacks.hpp"
#include "vcu_communicate.hpp"
#include "vcu_message.hpp"
#include "topic_publish.hpp"
#include "parse_argument.hpp"
#include <iostream>

//#include "My_topic_publish.hpp"
// Frequency 10Hz, period 100ms
#define FREQUNCY 10

// Static functions

static void quit_int(int signo);
static void quit_kill(int signo);
static void quit_term(int signo);
static void quit();
static void TimerCallBack(const ros::TimerEvent&);

// Pointer to node handle
std::shared_ptr<ros::NodeHandle> pNodeHandle;

// Pointer to TopicPublish
std::shared_ptr<TopicPublish> pTopicPublish;

//指向自定义消息发布类的指针
//std::shared_ptr<MyTopicPublish> MypTopicPublish;
std::shared_ptr<MyTopicPublish> MypTopicPublish;


//自定义的中转指令
extern geometry_msgs::Twist com_vel;

int main(int argc, char **argv)
{
    /*std::cout << "argc: " << argc << std::endl;
    if(!ParseArgument(argc, argv))
    {
        ROS_INFO("Argument error.");
        return EXIT_FAILURE;
    }
    if(argc == 2) UdpInterface::GetInstance().SetIpAndPort(argv[1]);
    else if(argc == 3) UdpInterface::GetInstance().SetIpAndPort(argv[1], atoi(argv[2]));*/

    int a = 0;
    ros::init(a, nullptr, "pix_robot_can_node");
    // while(ros::ok()){
       
    //     ROS_INFO("000000000000000000000000000000000");
    // }

    pNodeHandle = std::make_shared<ros::NodeHandle>();

    // Topics need to be subscribed.
    TopicCallbacks topicCallbacks;
    MyTopicCallbacks MytopicCallbacks; //自定义
    ros::Subscriber subSetMotorSpeed = pNodeHandle->subscribe("pix/set_motor_speed", 1, &TopicCallbacks::SetMotorSpeedCallback, &topicCallbacks);
    ros::Subscriber subSetMotorParameters = pNodeHandle->subscribe("pix/set_motor_parameters", 1, &TopicCallbacks::SetMotorParametersCallback, &topicCallbacks);
    ros::Subscriber subParkingBrake =  pNodeHandle->subscribe("pix/parking_brake", 1, &TopicCallbacks::ParkingBrakeCallback, &topicCallbacks);
    ros::Subscriber subCargoLock = pNodeHandle->subscribe("pix/cargo_unlock", 1, &TopicCallbacks::CargoUnlockCallback, &topicCallbacks);
    ros::Subscriber subLightControl = pNodeHandle->subscribe("pix/light_control", 1, &TopicCallbacks::LightControlCallback, &topicCallbacks);
    ros::Subscriber subDisplayOnLcd = pNodeHandle->subscribe("pix/display_on_lcd", 1, &TopicCallbacks::DisplayOnLcdCallback, &topicCallbacks);
    ros::Subscriber subResetMotor = pNodeHandle->subscribe("pix/reset_motor", 1, &TopicCallbacks::ResetMotorCallback, &topicCallbacks);
    ros::Subscriber subImpactProtect = pNodeHandle->subscribe("pix/impact_protect", 1, &TopicCallbacks::ImpactProtectCallback, &topicCallbacks);
    ros::Subscriber subSetRobotSpeed = pNodeHandle->subscribe("pix/set_robot_motion", 1, &TopicCallbacks::SetRobotMotionCallback, &topicCallbacks);
    //ros::Subscriber subGeometryTwist = pNodeHandle->subscribe("pix/com_vel", 1, &TopicCallbacks::GeometryTwistCallback, &topicCallbacks);
    ros::Subscriber subLatGeometryTwist = pNodeHandle->subscribe("pix/lat_com_vel", 1, &MyTopicCallbacks::LatGeometryTwistCallback, &MytopicCallbacks);
    ros::Subscriber subLonGeometryTwist = pNodeHandle->subscribe("pix/lon_com_vel", 1, &MyTopicCallbacks::LonGeometryTwistCallback, &MytopicCallbacks);
    ros::Subscriber subSetEmergency = pNodeHandle->subscribe("pix/set_emergency", 1, &TopicCallbacks::SetEmergencyCallback, &topicCallbacks);
    ros::Subscriber subSetAckermannMotion = pNodeHandle->subscribe("pix/set_ackermann_motion", 1, &TopicCallbacks::SetAckermannMotionCallback, &topicCallbacks);
    ros::Subscriber subSetSteeringMode = pNodeHandle->subscribe("pix/set_steering_mode", 1, &TopicCallbacks::SetSteeringModeCallback, &topicCallbacks);

    // Start threads
    pTopicPublish = std::make_shared<TopicPublish>(pNodeHandle);
    //自定义的消息发布
    MypTopicPublish =  std::make_shared<MyTopicPublish>(pNodeHandle);
    
    VcuCommunicate::GetInstance().StartThread();
    UdpInterface::GetInstance().StartThread();
    
    //---->>开启获取车辆状态线程 开启命令发送定时器
    /*pthread_t client_thread_id = start_status_monitor_thread();
    start_send_timer(20);*/

    // Register system exit signal processing functions for exit gracefully
    signal(SIGINT, quit_int);
    signal(SIGKILL, quit_kill);
    signal(SIGTERM, quit_term);

    ros::Rate loopRate(FREQUNCY);
    // Counter for lower frequency
    int freCounter = 0;

    // ROS multi-threaded spinning
    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();

    // ROS timer and callback
    ros::Timer timer = pNodeHandle->createTimer(ros::Duration(1), TimerCallBack);

    while (ros::ok())
    {
        //MypTopicPublish->Publish(com_vel);
        VcuCommunicate::GetInstance().SendHeartbeat();

        loopRate.sleep();
    }

    return EXIT_SUCCESS;
}

/**
 * \brief SIGINT process
 * \param signo Signal number type
 * \return none
 */
void quit_int(int signo)
{
    ROS_INFO("SIGINT");
    quit();
}

/**
 * \brief SIGKILL process
 * \param signo Signal number type
 * \return none
 */
void quit_kill(int signo)
{
    ROS_INFO("SIGKILL");
    quit();
}

/**
 * \brief SIGTERM process
 * \param signo Signal number type
 * \return none
 */
void quit_term(int signo)
{
    ROS_INFO("SIGTERM");
    quit();
}

/**
 * \brief Exit gracefully
 * \param none
 * \return none
 */
void quit()
{
    // Stop threads
    VcuCommunicate::GetInstance().StopThread();
    UdpInterface::GetInstance().StopThread();
    ROS_INFO("Program quit.");
    exit(EXIT_SUCCESS);
}

/**
 * \brief Timer call back function
 * \param ros::TimerEvent&
 * \return none
 */
void TimerCallBack(const ros::TimerEvent&)
{
    VcuCommunicate::GetInstance().SendHeartbeat();
}

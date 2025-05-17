#include <ros/ros.h>
#include <custom_msgs/TrafficLight.h>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

class UdpObuRecv
{
    private:
        struct obu_msg{
            int al_trffic_light_time;
            int al_trffic_light_ctl;
            int as_trffic_light_time;
            int as_trffic_light_ctl;
            int ar_trffic_light_time;
            int ar_trffic_light_ctl;

            int bl_trffic_light_time;
            int bl_trffic_light_ctl;
            int bs_trffic_light_time;
            int bs_trffic_light_ctl;
            int br_trffic_light_time;
            int br_trffic_light_ctl;

            obu_msg(){
                al_trffic_light_time = 0;
                al_trffic_light_ctl = 0;
                as_trffic_light_time = 0;
                as_trffic_light_ctl = 0;
                ar_trffic_light_time = 0;
                ar_trffic_light_ctl = 0;
                bl_trffic_light_time = 0;
                bl_trffic_light_ctl = 0;
                bs_trffic_light_time = 0;
                bs_trffic_light_ctl = 0;
                br_trffic_light_time = 0;
                br_trffic_light_ctl = 0;
            }
        };
    public:
        UdpObuRecv();
    public: // for ros
        bool onTrafficLightCallRecvd(custom_msgs::TrafficLight::Request  &req, custom_msgs::TrafficLight::Response &res);

    private:
        ros::NodeHandle nh;
        ros::ServiceServer service;

    public: // for udp
        void RecvUdpinit();
        void RecvUdp();

    private: // for udp
        struct obu_msg msg;
        int sockfd;
        int ret;
        socklen_t val;
};

bool UdpObuRecv::onTrafficLightCallRecvd(custom_msgs::TrafficLight::Request  &req,
                             custom_msgs::TrafficLight::Response &res)
{
    //if(fabs(req.direction + 90) < 0.1)
    //{
        res.color = msg.al_trffic_light_ctl;//color;
        res.time = msg.al_trffic_light_time;//time;
    //}
    return true;
}

void UdpObuRecv::RecvUdpinit()
{
    int size;
    struct sockaddr_in saddr;
    size = sizeof(struct sockaddr_in);
    bzero(&saddr,size);
    saddr.sin_family = AF_INET;
    saddr.sin_port = htons(6666);
    saddr.sin_addr.s_addr = htonl(INADDR_ANY);

    sockfd = socket(AF_INET,SOCK_DGRAM,0);
    if(sockfd<0){
        perror("socket failed");
    }
    ret = bind(sockfd,(struct sockaddr*)&saddr, sizeof(struct sockaddr));
    if(ret < 0){
        perror("sbind failed");
    }
    val = sizeof(struct sockaddr);
}

void UdpObuRecv::RecvUdp()
{
    struct sockaddr_in raddr;
    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        ret = recvfrom(sockfd, &msg, sizeof(msg), 0, (struct sockaddr*)&raddr, &val);
        //std::cout << "---" << msg.al_trffic_light_ctl<<"  "<<msg.al_trffic_light_time << std::endl;
        loop_rate.sleep();
    }
}

UdpObuRecv::UdpObuRecv()
{
    service = nh.advertiseService("traffic_light_service", &UdpObuRecv::onTrafficLightCallRecvd, this);
    RecvUdpinit();
    boost::function<void()> thred1 =  boost::bind(&UdpObuRecv::RecvUdp,this);
    boost::thread thrd(thred1);
    ros::spin();
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "obu_node");
    UdpObuRecv udp;
    return 0;
}

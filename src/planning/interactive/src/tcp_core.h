#ifndef TCP_CORE_H
#define TCP_CORE_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <fstream>
using namespace std;
class tcp_core{

private:
    int  sockfd;
    //struct sockaddr_in loc_addr;    //用于指定本地监听信息
    struct sockaddr_in des_addr;    //獲取客戶端地址信息
    socklen_t loc_addr_len,des_addr_len;
    string address;
    int port;
public:
    int socket_init();
    void socket_close();
    int send_data(char *send_buf,int size, int flags);
    int recv_data(char *recv_buf,int size, int flags);
    void ipfigFileRead();
public:
    tcp_core(){}
};

/***************************************************************************
 * @brief 初始化
 * @param 
 */
int tcp_core::socket_init(){

    //loc_addr_len = sizeof(struct sockaddr);
    //des_addr_len = sizeof(struct sockaddr);


    // //设置本地地址信息，ip信息
    // bzero(&loc_addr,sizeof(struct sockaddr_in));
    // loc_addr.sin_family = AF_INET;
    // loc_addr.sin_port = htons(6678);
    // loc_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    //设置目的地址信息，ip信息
    bzero(&des_addr,sizeof(struct sockaddr_in));
    des_addr.sin_family = AF_INET;
    des_addr.sin_port = htons(port);
    des_addr.sin_addr.s_addr = inet_addr(address.c_str());
    //des_addr.sin_port = htons(6188);
    //des_addr.sin_addr.s_addr = inet_addr("39.103.205.4");
    //des_addr.sin_addr.s_addr = inet_addr("10.16.10.124");

    //创建tcp套接字
    sockfd = socket(AF_INET,SOCK_STREAM,0);
    if(sockfd<0){
        perror("to UI tcp socket failed");
        return -1;
    }


    // 设置超时
    struct timeval timeout;
    timeout.tv_sec = 1;//秒
    timeout.tv_usec = 0;//微秒
    if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) == -1) {
        perror("to UI tcp setsockopt failed");
    }
    
    // //设置端口复用
    // int on = 1;
    // setsockopt(sockfd,SOL_SOCKET,SO_REUSEADDR,&on,sizeof(on));

    // //绑定地址信息，ip信息
    // int ret = 0;
    // ret = bind(sockfd,(struct sockaddr*)&des_addr, sizeof(struct sockaddr));
    // if(ret < 0){
    //     perror("socket bind failed");
    //     return -1;
    // }

    //连接请求
    if(connect(sockfd, (struct sockaddr*)&des_addr, sizeof(struct sockaddr)) == -1) {
        perror("to UI tcp connect server failed");
        return -1;
    }

    return 0; 
}
void tcp_core::socket_close()
{
    close(sockfd);
}
/******************************************************************************************************
 * @brief 数据发送
 * @param flags: 默认为0-阻塞式发送
 */
int tcp_core::send_data(char *send_buf,int size, int flags){
    int ret = 0;
    ret = send(sockfd,send_buf,size,flags);
    if(ret==-1)
    {
        printf("to UI tcp send data failure\n");
        return -1;
    }else if(ret == 0) {
        printf("to UI tcp connect closed\n");
        return 0;
    }
    return ret;
}

/********************************************************************************************************
 * @brief 数据接收
 * @param flags: 默认为0-阻塞式接收
 */
int tcp_core::recv_data(char *recv_buf,int size, int flags){
    int ret = 0;
    //ret = recvfrom(sockfd,recv_buf,size,flags,(struct sockaddr*)&des_addr, &des_addr_len);
    ret = recv(sockfd,recv_buf,size,MSG_WAITALL);
    // printf("size: %d\n",size);
    // printf("ret: %d\n",ret);
    if(ret==-1)
    {
        printf("to UI tcp recv data failure\n");
        // printf("%d\n", errno);
        return -1;
    }else if(ret == 0) {
        printf("to UI tcp connect closed\n");
        return 0;
    }
    return ret;
}
void tcp_core::ipfigFileRead()
{
    ifstream configFile;
    //const char* env_p = std::getenv("SEED_HOME");
    string path = "/home/nvidia/Desktop/slam_indoor_1116_qiehuan_v2";
    path += "/ip.conf";
    configFile.open(path.c_str());
    string strLine;
    if (configFile.is_open())
    {
        while (!configFile.eof())
        {
            getline(configFile, strLine);
            size_t pos = strLine.find('=');
            string key = strLine.substr(0, pos);

            if (key == "address") {
                address = strLine.substr(pos + 1);
                //cout<<"address :"<<address<<endl;
            }
            else if (key == "port") {
                port = atoi(strLine.substr(pos + 1).c_str());
                //cout<<"port :"<<port<<endl;
                //port1=atoi(port.c_str());
            }
        }
    }
    else
    {
        cout << "Cannot open config file!" << endl;
    }
}
#endif //TCP_CORE_H

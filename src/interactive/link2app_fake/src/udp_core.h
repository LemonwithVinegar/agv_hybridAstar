#ifndef UDP_CORE_H
#define UDP_CORE_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <iostream>

class udp_core{

private:
    int  sockfd;
    struct sockaddr_in loc_addr;    //用于指定本地监听信息
    struct sockaddr_in des_addr;    //獲取客戶端地址信息
    socklen_t loc_addr_len,des_addr_len;

public:
    int socket_init(uint32_t loc_port, char* des_ip, uint32_t des_port, float time_out);
    int send_data(char *send_buf,int size);
    int recv_data(char *recv_buf,int size);
public:
    udp_core(){}
};

/***************************************************************************
 * @brief 初始化
 * @param loc_port:本地端口
 *        des_ip  ：目的IP
 *        des_port：目的端口
 *        time_out：超时时间，为0时默认阻塞,单位秒     
 */
int udp_core::socket_init(uint32_t loc_port, char* des_ip, uint32_t des_port, float time_out){

    loc_addr_len = sizeof(struct sockaddr);
    des_addr_len = sizeof(struct sockaddr);

    //设置本地地址信息，ip信息
    bzero(&loc_addr,sizeof(struct sockaddr_in));
    loc_addr.sin_family = AF_INET;
    loc_addr.sin_port = htons(loc_port);
    loc_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    //设置目的地址信息，ip信息
    bzero(&des_addr,sizeof(struct sockaddr_in));
    des_addr.sin_family = AF_INET;
    des_addr.sin_port = htons(des_port);
    des_addr.sin_addr.s_addr = inet_addr(des_ip);

    //创建udp套接字
    sockfd = socket(AF_INET,SOCK_DGRAM,0);
    if(sockfd<0){
        perror("socket failed");
        return -1;
    }
    
    //设置端口复用
    int on = 1;
    if (setsockopt(sockfd,SOL_SOCKET,SO_REUSEADDR,&on,sizeof(on)) == -1) {
        perror("setsockopt timeout failed");
    }

    //设置接收超时(与设置非阻塞不共用)
    if(time_out != 0){
        struct timeval timeout;
        timeout.tv_sec  = (int)time_out;  //秒
        timeout.tv_usec = (int)( (time_out - (int)time_out) * 1000);   //微秒
        if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) == -1) {
            perror("setsockopt timeout failed");
        }
    }
    
    //设置非阻塞(与设置超时不共用)()
/*     int flag = fcntl(sockfd, F_GETFL, 0);
    if (flag < 0) {
        perror("fcntl F_GETFL fail");
        return;
    }
    if (fcntl(sockfd, F_SETFL, flag | O_NONBLOCK) < 0) {
        perror("fcntl F_SETFL fail");
    } */

    //绑定地址信息，ip信息
    int ret = 0;
    ret = bind(sockfd,(struct sockaddr*)&loc_addr, sizeof(struct sockaddr));
    if(ret < 0){
        perror("sbind failed");
        return -1;
    }
    return 0; 
}

/******************************************************************************************************
 * @brief 数据发送
 * @param 
 */
int udp_core::send_data(char *send_buf,int size){
    int ret = 0;
    ret = sendto(sockfd,send_buf,size,0,(struct sockaddr*)&des_addr, des_addr_len);
    if(ret==-1)
    {
        printf("send data failure\n");
        return -1;
    }
    return ret;
}

/********************************************************************************************************
 * @brief 数据接收
 * @param 
 */
int udp_core::recv_data(char *recv_buf,int size){
    int ret = 0;
    ret = recvfrom(sockfd,recv_buf,size,0,(struct sockaddr*)&des_addr, &des_addr_len);
    if(ret==-1)
    {
       // printf("recv data failure\n");
        return -1;
    }
    return ret;
}

#endif //UDP_CORE_H
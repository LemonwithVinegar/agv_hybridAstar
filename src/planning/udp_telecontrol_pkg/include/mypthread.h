#ifndef MYPTHREAD_H
#define MYPTHREAD_H

#include <pthread.h>
#include <iostream>

//自定义线程类（用C语言写）
class mypthread{
    private:
        pthread_t tid;
    public:
        void create(void* threadHandlerFunc(void* arg));
};

void mypthread::create(void* threadHandlerFunc(void* arg)){
    int ret = pthread_create(&(this->tid), NULL, threadHandlerFunc, NULL);
    if(ret != 0){
        // perror("pthread create error");
        ROS_ERROR_STREAM_THROTTLE(1,"pthread create error");
        exit(1);
    }
}










#endif
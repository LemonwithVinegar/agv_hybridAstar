#include "parse_argument.hpp"
#include <ros/ros.h>

char ipAddr[16];
uint16_t udpPort;

bool ParseIP(char* arg)
{
    int first = 0, index = 0, dotNum = 0;
    while(true)
    {
        char ch = arg[index];
        if(ch == '.')
        {
            if(index - first > 3 || index == first) return false;
            first = index+1;
            if(++dotNum > 3) return false;
        }
        else if(ch == '\0')
        {
            if(dotNum != 3) return false;
            else if(index > 15) return false;
            else if(arg[index-1] < '0' || arg[index-1] > '9') return false;
            break;
        }
        else if(ch < '0' || ch > '9') return false;
        index++;
    }
    return true;
}

bool ParsePort(char* arg)
{
    int index = 0;
    while(true)
    {
        char ch = arg[index++];
        if(index > 6) return false;
        else if(ch == '\0') break;
        else if('9' < ch || ch < '0') return false;
    }
    return true;
}

bool ParseArgument(int argc, char **argv)
{
    if (argc == 1)
        return true;
    else if (argc == 2)
    {
        ROS_INFO("%s",argv[1]);
        char *arg1 = argv[1];
        return ParseIP(arg1);
    }
    else if (argc == 3)
    {
        char *arg1 = argv[1], *arg2 = argv[2];
        return (ParseIP(arg1) && ParsePort(arg2));
    }
    else
        return false;
    return true;
}

char *GetIpAddr(void)
{
}

uint16_t GetPort(void)
{
}
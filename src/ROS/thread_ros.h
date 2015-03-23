#ifndef THREAD_ROS_H
#define THREAD_ROS_H

#include <qthread.h>
#include <iostream>
#include <sys/time.h>

#include <ros/ros.h>
#include "../shared_memory.h"
#include <mavros/OverrideRCIn.h>
#include <mavros/ParamGet.h>

class Thread_ROS:public QThread
{
public:
    Thread_ROS(Shared_memory *share_memory);
private:
    Shared_memory* share_memory;
    ros::Publisher rc_override_pub;
    ros::ServiceClient cl_param;

    int RC_Param(std::string s, int i);

protected:
    void run();
};

#endif // THREAD_ROS_H

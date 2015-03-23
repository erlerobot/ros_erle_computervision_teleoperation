#include <iostream>

#include <QApplication>

//ROS
#include <ros/ros.h>

#include "shared_memory.h"

#include "ROS/subscribe_mavros_state.h"
#include "ROS/mavros_setstreamrate.h"
#include "ROS/thread_ros.h"
#include "ROS/ros_image.h"

#include "ComputerVision/thread_processing.h"

#include "gui/threadgui.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "ros_control_drone");
    QApplication a(argc, argv);

    ros::NodeHandle n;
    Shared_memory* share_memory = new Shared_memory();

    Subscribe_mavros_state mavros_state;

    ros::Subscriber mavros_state_sub = n.subscribe("/mavros/state",
                                                   1,
                                                   &Subscribe_mavros_state::mavrosStateCb,
                                                   &mavros_state);

    Thread_Processing thread_processing(share_memory);
    thread_processing.start();

    MAVROS_setStreamRate setStreamRate;
    ROS_image ros_image(share_memory);

    Thread_ROS t_ros(share_memory);
    t_ros.start();

    threadGUI t_gui(share_memory);
    t_gui.start();

    return a.exec();
}


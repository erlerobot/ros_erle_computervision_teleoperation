#ifndef ROS_IMAGE_H
#define ROS_IMAGE_H

//ROS
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

//OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "../shared_memory.h"

class ROS_image
{
public:
    ROS_image(Shared_memory* shared_memory);
    ~ROS_image();

    void imageCb(const sensor_msgs::ImageConstPtr& msg);
    void imageCompressCb(const sensor_msgs::ImageConstPtr& msg);
    cv::Mat getImage();
private:

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Subscriber image_sub_compress_;
    image_transport::Publisher image_pub_;

    Shared_memory* shared_memory;
};

#endif // ROS_IMAGE_H

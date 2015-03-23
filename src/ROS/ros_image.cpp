#include "ros_image.h"

ROS_image::ROS_image(Shared_memory* shared_memory): it_(nh_)
{
    // Subscrive to input video feed and publish output video feed
//    image_sub_ = it_.subscribe("image_raw", 1, &ROS_image::imageCb, this);

    this->shared_memory = shared_memory;
    image_sub_compress_ = it_.subscribe("image_raw_compress", 1, &ROS_image::imageCompressCb, this);

}

ROS_image::~ROS_image()
{
}

void ROS_image::imageCompressCb(const sensor_msgs::ImageConstPtr& msg){
    cv_bridge::CvImagePtr cv_ptr;
    try{
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    }catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat desCompress;
    desCompress = cv::imdecode( cv_ptr->image, CV_LOAD_IMAGE_COLOR);
    cv::resize(desCompress, desCompress, cv::Size(320, 240));
    cv::cvtColor(desCompress, desCompress, CV_BGR2RGB);
    shared_memory->setImage(desCompress);
}

void ROS_image::imageCb(const sensor_msgs::ImageConstPtr& msg){
    cv_bridge::CvImagePtr cv_ptr;
    try{
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    shared_memory->setImage(cv_ptr->image);
}

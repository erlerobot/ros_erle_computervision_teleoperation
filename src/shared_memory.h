#ifndef SHARED_MEMORY_H
#define SHARED_MEMORY_H

//OpenCV
#include <opencv2/core/core.hpp>

//Standard
#include <iostream>

class Shared_memory
{
public:
    Shared_memory();

    void update();

    void setImage(cv::Mat& im);
    cv::Mat getImage();

    void setImage_filtered_rgb(cv::Mat& im);
    cv::Mat getImage_filtered_rgb();

    void setImage_filtered_hsv(cv::Mat& im);
    cv::Mat getImage_filtered_hsv();

    void setImage_hsv(cv::Mat& im);
    cv::Mat getImage_hsv();

    void setImage_rgb(cv::Mat& im);
    cv::Mat getImage_rgb();

    void setColorFilter(double hmax, double hmin, double smax, double smin, double vmax, double vmin,
                        int rmax, int rmin, int gmax, int gmin, int bmax, int bmin);

    void setPitch(int var);
    void setRoll(int var);
    void setYaw(int var);
    void setThrottle(int var);

    int getPitch();
    int getRoll();
    int getYaw();
    int getThrottle();

    void setAuto_Pitch(int var);
    void setAuto_Roll(int var);
    void setAuto_Yaw(int var);
    void setAuto_Throttle(int var);

    int getAuto_Pitch();
    int getAuto_Roll();
    int getAuto_Yaw();
    int getAuto_Throttle();

    double getHMax();
    double getSMax();
    double getVMax();
    int getRMax();
    int getGMax();
    int getBMax();

    double getHMin();
    double getSMin();
    double getVMin();
    int getRMin();
    int getGMin();
    int getBMin();

    void setRC_maxlimits(std::vector<int>);
    void setRC_minlimits(std::vector<int>);

    std::vector<int> getRC_maxlimits();
    std::vector<int> getRC_minlimits();


private:
    cv::Mat image;
    cv::Mat frame_filtered_rgb;
    cv::Mat frame_filtered_hsv;
    cv::Mat hsv_image_temp;
    cv::Mat rgb_image_temp;

    pthread_mutex_t mutex;

    double hmax;
    double hmin;
    double smax;
    double smin;
    double vmax;
    double vmin;
    int rmax;
    int rmin;
    int gmax;
    int gmin;
    int bmax;
    int bmin;

    int roll;
    int pitch;
    int yaw;
    int throttle;

    int auto_roll;
    int auto_pitch;
    int auto_yaw;
    int auto_throttle;

    std::vector<int> rc_max_limits;
    std::vector<int> rc_min_limits;
};

#endif // SHARED_MEMORY_H

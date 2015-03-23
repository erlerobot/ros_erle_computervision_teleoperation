#include "shared_memory.h"

Shared_memory::Shared_memory()
{
    if (pthread_mutex_init(&mutex, NULL) != 0){
        std::cout << "mutex init failed" << std::endl;
    }
}

void Shared_memory::update()
{

}

void Shared_memory::setRC_maxlimits(std::vector<int> v)
{
    pthread_mutex_lock( &mutex );
    this->rc_max_limits = v;
    pthread_mutex_unlock( &mutex );
}

void Shared_memory::setRC_minlimits(std::vector<int> v)
{
    pthread_mutex_lock( &mutex );
    this->rc_min_limits = v;
    pthread_mutex_unlock( &mutex );
}

std::vector<int> Shared_memory::getRC_maxlimits()
{
    std::vector<int> result;
    pthread_mutex_lock( &mutex );
    result = this->rc_max_limits;
    pthread_mutex_unlock( &mutex );
    return result;
}

std::vector<int> Shared_memory::getRC_minlimits()
{
    std::vector<int> result;
    pthread_mutex_lock( &mutex );
    result = this->rc_min_limits;
    pthread_mutex_unlock( &mutex );
    return result;
}

void Shared_memory::setPitch(int var)
{
    pthread_mutex_lock( &mutex );
    this->pitch = var;
    pthread_mutex_unlock( &mutex );
}

void Shared_memory::setRoll(int var)
{
    pthread_mutex_lock( &mutex );
    this->roll = var;
    pthread_mutex_unlock( &mutex );
}

void Shared_memory::setYaw(int var)
{
    pthread_mutex_lock( &mutex );
    this->yaw = var;
    pthread_mutex_unlock( &mutex );
}
void Shared_memory::setThrottle(int var)
{
    pthread_mutex_lock( &mutex );
    this->throttle = var;
    pthread_mutex_unlock( &mutex );
}

int Shared_memory::getPitch()
{
    int result;
    pthread_mutex_lock( &mutex );
    result = this->pitch;
    pthread_mutex_unlock( &mutex );
    return result;
}

int Shared_memory::getRoll()
{
    int result;
    pthread_mutex_lock( &mutex );
    result = this->roll;
    pthread_mutex_unlock( &mutex );
    return result;
}

int Shared_memory::getYaw()
{
    int result;
    pthread_mutex_lock( &mutex );
    result = this->yaw;
    pthread_mutex_unlock( &mutex );
    return result;
}

int Shared_memory::getThrottle()
{
    int result;
    pthread_mutex_lock( &mutex );
    result = this->throttle;
    pthread_mutex_unlock( &mutex );
    return result;
}

void Shared_memory::setAuto_Pitch(int var)
{
    pthread_mutex_lock( &mutex );
    this->auto_pitch = var;
    pthread_mutex_unlock( &mutex );
}

void Shared_memory::setAuto_Roll(int var)
{
    pthread_mutex_lock( &mutex );
    this->auto_roll = var;
    pthread_mutex_unlock( &mutex );
}

void Shared_memory::setAuto_Yaw(int var)
{
    pthread_mutex_lock( &mutex );
    this->auto_yaw = var;
    pthread_mutex_unlock( &mutex );
}
void Shared_memory::setAuto_Throttle(int var)
{
    pthread_mutex_lock( &mutex );
    this->auto_throttle = var;
    pthread_mutex_unlock( &mutex );
}

int Shared_memory::getAuto_Pitch()
{
    int result;
    pthread_mutex_lock( &mutex );
    result = this->auto_pitch;
    pthread_mutex_unlock( &mutex );
    return result;
}

int Shared_memory::getAuto_Roll()
{
    int result;
    pthread_mutex_lock( &mutex );
    result = this->auto_roll;
    pthread_mutex_unlock( &mutex );
    return result;
}

int Shared_memory::getAuto_Yaw()
{
    int result;
    pthread_mutex_lock( &mutex );
    result = this->auto_yaw;
    pthread_mutex_unlock( &mutex );
    return result;
}

int Shared_memory::getAuto_Throttle()
{
    int result;
    pthread_mutex_lock( &mutex );
    result = this->auto_throttle;
    pthread_mutex_unlock( &mutex );
    return result;
}

void Shared_memory::setImage(cv::Mat& im)
{
    pthread_mutex_lock( &mutex );
    im.copyTo(this->image);
    pthread_mutex_unlock( &mutex );
}

cv::Mat Shared_memory::getImage()
{
    cv::Mat result;
    pthread_mutex_lock( &mutex );
    this->image.copyTo(result);
    pthread_mutex_unlock( &mutex );
    return result;
}

void Shared_memory::setImage_filtered_rgb(cv::Mat& im)
{
    pthread_mutex_lock( &mutex );
    im.copyTo(this->frame_filtered_rgb);
    pthread_mutex_unlock( &mutex );
}

cv::Mat Shared_memory::getImage_filtered_rgb()
{
    cv::Mat result;
    pthread_mutex_lock( &mutex );
    this->frame_filtered_rgb.copyTo(result);
    pthread_mutex_unlock( &mutex );
    return result;
}

void Shared_memory::setImage_filtered_hsv(cv::Mat& im)
{
    pthread_mutex_lock( &mutex );
    im.copyTo(this->frame_filtered_hsv);
    pthread_mutex_unlock( &mutex );
}

cv::Mat Shared_memory::getImage_filtered_hsv()
{
    cv::Mat result;
    pthread_mutex_lock( &mutex );
    this->frame_filtered_hsv.copyTo(result);
    pthread_mutex_unlock( &mutex );
    return result;
}

void Shared_memory::setImage_hsv(cv::Mat& im)
{
    pthread_mutex_lock( &mutex );
    im.copyTo(this->hsv_image_temp);
    pthread_mutex_unlock( &mutex );
}

cv::Mat Shared_memory::getImage_hsv()
{
    cv::Mat result;
    pthread_mutex_lock( &mutex );
    this->hsv_image_temp.copyTo(result);
    pthread_mutex_unlock( &mutex );
    return result;
}

void Shared_memory::setImage_rgb(cv::Mat& im)
{
    pthread_mutex_lock( &mutex );
    im.copyTo(this->rgb_image_temp);
    pthread_mutex_unlock( &mutex );
}

cv::Mat Shared_memory::getImage_rgb()
{
    cv::Mat result;
    pthread_mutex_lock( &mutex );
    this->rgb_image_temp.copyTo(result);
    pthread_mutex_unlock( &mutex );
    return result;
}

void Shared_memory::setColorFilter(double hmax, double hmin, double smax, double smin, double vmax, double vmin,
                                   int rmax, int rmin, int gmax, int gmin, int bmax, int bmin)
{
    pthread_mutex_lock( &mutex );
    this->hmax = hmax;
    this->hmin = hmin;
    this->smax = smax;
    this->smin = smin;
    this->vmax = vmax;
    this->vmin = vmin;
    this->rmax = rmax;
    this->rmin = rmin;
    this->gmax = gmax;
    this->gmin = gmin;
    this->bmax = bmax;
    this->bmin = bmin;
    pthread_mutex_unlock( &mutex );
}

double Shared_memory::getHMax()
{
    double result;
    pthread_mutex_lock( &mutex );
    result = this->hmax;
    pthread_mutex_unlock( &mutex );
    return result;
}

double Shared_memory::getHMin()
{
    double result;
    pthread_mutex_lock( &mutex );
    result = this->hmin;
    pthread_mutex_unlock( &mutex );
    return result;
}

double Shared_memory::getSMax()
{
    double result;
    pthread_mutex_lock( &mutex );
    result = this->smax;
    pthread_mutex_unlock( &mutex );
    return result;
}

double Shared_memory::getSMin()
{
    double result;
    pthread_mutex_lock( &mutex );
    result = this->smin;
    pthread_mutex_unlock( &mutex );
    return result;
}

double Shared_memory::getVMax()
{
    double result;
    pthread_mutex_lock( &mutex );
    result = this->vmax;
    pthread_mutex_unlock( &mutex );
    return result;
}

double Shared_memory::getVMin()
{
    double result;
    pthread_mutex_lock( &mutex );
    result = this->vmin;
    pthread_mutex_unlock( &mutex );
    return result;
}

int Shared_memory::getRMax()
{
    int result;
    pthread_mutex_lock( &mutex );
    result = this->rmax;
    pthread_mutex_unlock( &mutex );
    return result;
}

int Shared_memory::getRMin()
{
    int result;
    pthread_mutex_lock( &mutex );
    result = this->rmin;
    pthread_mutex_unlock( &mutex );
    return result;
}

int Shared_memory::getGMax()
{
    int result;
    pthread_mutex_lock( &mutex );
    result = this->gmax;
    pthread_mutex_unlock( &mutex );
    return result;
}

int Shared_memory::getGMin()
{
    int result;
    pthread_mutex_lock( &mutex );
    result = this->gmin;
    pthread_mutex_unlock( &mutex );
    return result;
}

int Shared_memory::getBMax()
{
    int result;
    pthread_mutex_lock( &mutex );
    result = this->bmax;
    pthread_mutex_unlock( &mutex );
    return result;
}

int Shared_memory::getBMin()
{
    int result;
    pthread_mutex_lock( &mutex );
    result = this->bmin;
    pthread_mutex_unlock( &mutex );
    return result;
}

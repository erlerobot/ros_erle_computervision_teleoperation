#ifndef THREAD_PROCESSING_H
#define THREAD_PROCESSING_H

#include <QThread>
#include "../shared_memory.h"
#include <sys/time.h>
#include "colorfilter.h"
#include "kalman2d.h"

class Thread_Processing: public QThread
{
public:
    Thread_Processing(Shared_memory* shared_memory);

private:
    Shared_memory* shared_memory;
    cv::Mat hsv_image;

    Kalman2D* kalman_RGB;
    Kalman2D* kalman_HSV;

    float x_rgb;
    float y_rgb;

    float x_hsv;
    float y_hsv;

    int auto_roll;
    int auto_pitch;
    int auto_yaw;
    int auto_throttle;

protected:
    void run();
};

#endif // THREAD_PROCESSING_H

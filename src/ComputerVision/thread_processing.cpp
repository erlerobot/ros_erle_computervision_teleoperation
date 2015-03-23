#include "thread_processing.h"

Thread_Processing::Thread_Processing(Shared_memory *shared_memory)
{
    this->shared_memory = shared_memory;
    hsv_image = ColorFilter::draw_hsvmap(320);

    kalman_RGB = new Kalman2D();
    kalman_HSV = new Kalman2D();

    x_rgb=0;
    y_rgb=0;
    y_hsv=0;
    x_hsv=0;

    auto_roll = 250;
    auto_pitch = 250;
    auto_yaw = 250;
    auto_throttle = 250;
}

void Thread_Processing::run()
{
    struct timeval a, b;
    long totalb, totala;
    long diff;

    int cycle_processing = 20;

    cv::Mat frame_filtered_rgb;
    cv::Mat frame_filtered_hsv;

    while (true) {
        gettimeofday(&a, NULL);
        totala = a.tv_sec * 1000000 + a.tv_usec;

        //
        cv::Mat frame = shared_memory->getImage();

        if(frame.data==0)
            continue;

        frame.copyTo(frame_filtered_rgb);
        frame.copyTo(frame_filtered_hsv);
        cv::Mat hsv_image_temp;
        cv::Mat rgb_image_temp;
        hsv_image.copyTo(hsv_image_temp);
        hsv_image.copyTo(rgb_image_temp);

        ColorFilter::filterHSV_RGB(frame,
                                   frame_filtered_rgb,
                                   frame_filtered_hsv,
                                   rgb_image_temp,
                                   hsv_image_temp,
                                   shared_memory->getHMax(),
                                   shared_memory->getHMin(),
                                   shared_memory->getSMax(),
                                   shared_memory->getSMin(),
                                   shared_memory->getVMax(),
                                   shared_memory->getVMin(),
                                   shared_memory->getRMax(),
                                   shared_memory->getRMin(),
                                   shared_memory->getGMax(),
                                   shared_memory->getGMin(),
                                   shared_memory->getBMax(),
                                   shared_memory->getBMin());

        hsv_image_temp = ColorFilter::drawcheese(hsv_image_temp,
                                                 shared_memory->getHMax(),
                                                 shared_memory->getHMin(),
                                                 shared_memory->getSMax(),
                                                 shared_memory->getSMin());

        int num_pixel_hsv = ColorFilter::blobDetection(frame_filtered_hsv, kalman_HSV, x_hsv, y_hsv);
        ColorFilter::blobDetection(frame_filtered_rgb, kalman_RGB, x_rgb, y_rgb);

//        cv::circle(frame_filtered_hsv, cv::Point(x_hsv, y_hsv), 3, cv::Scalar(255, 0, 0));

        auto_roll = 250*((x_hsv) / (320/2));

        if(num_pixel_hsv < 200){
            auto_pitch = 100;
        }else if(num_pixel_hsv > 2000){
            auto_pitch = 300;
        }else{
            auto_pitch = 250;
        }

        auto_throttle = 250*((y_hsv) / (240/2));


        shared_memory->setAuto_Pitch(auto_pitch);
        shared_memory->setAuto_Roll(auto_roll);
        shared_memory->setAuto_Throttle(auto_throttle);
        shared_memory->setAuto_Yaw(auto_yaw);

        shared_memory->setImage_filtered_rgb(frame_filtered_rgb);
        shared_memory->setImage_filtered_hsv(frame_filtered_hsv);
        shared_memory->setImage_hsv(hsv_image_temp);
        shared_memory->setImage_rgb(rgb_image_temp);
        //

        gettimeofday(&b, NULL);
        totalb = b.tv_sec * 1000000 + b.tv_usec;
        diff = (totalb - totala) / 1000;

        if (diff < 0 || diff > cycle_processing)
            diff = cycle_processing;
        else
            diff = cycle_processing - diff;

        /*Sleep Algorithm*/
        usleep(diff * 1000);
    }

}

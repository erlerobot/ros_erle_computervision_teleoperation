#ifndef COLORFILTER_H
#define COLORFILTER_H

//Opencv
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "kalman2d.h"

#define DEGTORAD     (3.14159264 / 180.0)
#define PI 3.141592654
#define RADTODEG     (180.0 /3.14159264)

class ColorFilter
{
public:
    ColorFilter();

public:
    static cv::Mat draw_hsvmap(int size);
    static void hsv2rgb(double H, double S, double V, double *r, double *g, double *b);
    static double getH(double r, double g, double b);
    static double getS(double r, double g, double b);
    static double getV(double r, double g, double b);

    static int drawarc(cv::Mat& image, int xcentro, int ycentro, int radio, int x1, int y1, int x2, int y2);
    static int drawcircle(cv::Mat& image, int xcentro, int ycentro, int radio);
    static int lineinimage(cv::Mat& image, int xa, int ya, int xb, int yb);


    static cv::Mat drawcheese(cv::Mat& hsv_image, double sliderH_max, double sliderH_min, double sliderS_max, double sliderS_min);
    static cv::Mat filterRGB(cv::Mat &frame, cv::Mat& rgb_image, int rmax, int rmin, int gmax, int gmin, int bmax, int bmin);
    static cv::Mat filterHSV(cv::Mat& frame, cv::Mat& hsv_image, double hmax, double hmin, double smax, double smin, double vmax, double vmin);
    static void filterHSV_RGB(cv::Mat& frame, cv::Mat& rgb_filtered, cv::Mat& hsv_filtered, cv::Mat& rgb_image, cv::Mat& hsv_image,
                                double hmax, double hmin, double smax, double smin, double vmax, double vmin,
                                int rmax, int rmin, int gmax, int gmin, int bmax, int bmin);

    static int blobDetection(cv::Mat &frame_filtered, Kalman2D* kalman, float& x, float& y);
};

#endif // COLORFILTER_H

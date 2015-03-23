#ifndef GUI_H
#define GUI_H

//Qt
#include <QtGui>

//standard
#include <iostream>

//ROS
#include <ros/ros.h>

//OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "../shared_memory.h"
#include "../ComputerVision/colorfilter.h"

#include "RC_Widget.h"

class GUI:public QWidget
{
    Q_OBJECT

public:
    GUI(Shared_memory* share_memory);
    void updateThreadGUI();


private:
    Shared_memory* share_memory;
    void config_Image_GUI();
    void config_color_filter();
    void config_RC();

    QGridLayout* mainLayout;
    QGridLayout* layImage_original;

    RC_Widget* channel12;
    RC_Widget* channel34;
    QCheckBox* checkbox_automatic;

    QLabel* label_image;
    QLabel* label_filtered_rgb;
    QLabel* label_filtered_hsv;
    QLabel* label_filtered_rgb_temp;
    QLabel* label_filtered_hsv_temp;

    ///
    QLabel* labelImage_filtered_hsv;
    QLabel* labelImage_filtered_rgb;
    QLabel* labelImage_hsv;
    QLabel* labelImage_rgb;

    QSlider* sliderR_min;
    QSlider* sliderB_min;
    QSlider* sliderG_min;
    QSlider* sliderR_max;
    QSlider* sliderB_max;
    QSlider* sliderG_max;
    QLabel* textR_min;
    QLabel* textG_min;
    QLabel* textB_min;
    QLabel* textR_max;
    QLabel* textG_max;
    QLabel* textB_max;
    QLabel* textR_value_min;
    QLabel* textG_value_min;
    QLabel* textB_value_min;
    QLabel* textR_value_max;
    QLabel* textG_value_max;
    QLabel* textB_value_max;

    QDoubleSpinBox* sliderH_min;
    QDoubleSpinBox* sliderS_min;
    QDoubleSpinBox* sliderV_min;
    QDoubleSpinBox* sliderH_max;
    QDoubleSpinBox* sliderS_max;
    QDoubleSpinBox* sliderV_max;
    QLabel* textH_min;
    QLabel* textS_min;
    QLabel* textV_min;
    QLabel* textH_max;
    QLabel* textS_max;
    QLabel* textV_max;
    QLabel* textH_value_max;
    QLabel* textS_value_max;
    QLabel* textV_value_max;
    QLabel* textH_value_min;
    QLabel* textS_value_min;
    QLabel* textV_value_min;

    QCheckBox* checkRGB;
    QCheckBox* checkHSV;

    float x_rgb;
    float y_rgb;

    float x_hsv;
    float y_hsv;
    cv::Mat frame;

    cv::Mat hsv_image;
    cv::Mat frame_filtered_rgb;
    cv::Mat frame_filtered_hsv;

    QLabel* label_pitch;
    QLabel* label_roll;
    QLabel* label_throttle;
    QLabel* label_yaw;

    ///
signals:
    void signal_updateGUI();

public slots:
    void on_updateGUI_recieved();
    void on_checkbox_automatic_changed();
    void mousePressEvent(QMouseEvent *event);
};

#endif // GUI_H

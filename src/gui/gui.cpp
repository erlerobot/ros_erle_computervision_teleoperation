#include "gui.h"

GUI::GUI(Shared_memory *share_memory)
{
    this->share_memory = share_memory;

    mainLayout = new QGridLayout();

    config_Image_GUI();

    config_color_filter();

    config_RC();

    setLayout(mainLayout);

    setVisible(true);

    connect(this, SIGNAL(signal_updateGUI()), this, SLOT(on_updateGUI_recieved()));

    show();
}

void GUI::config_RC()
{
    QGroupBox* boxRC = new QGroupBox("Radio Control");
    QGridLayout* layRC = new QGridLayout();
    boxRC->setLayout(layRC);

    std::vector<int> rc_maxlimits = share_memory->getRC_maxlimits();
    std::vector<int> rc_minlimits = share_memory->getRC_minlimits();

    channel12 = new RC_Widget(rc_maxlimits[0], rc_minlimits[0], rc_maxlimits[1], rc_minlimits[1], 1, -1);
    channel34 = new RC_Widget(rc_maxlimits[2], rc_minlimits[2], rc_maxlimits[3], rc_minlimits[3], 1, 1);

    label_pitch = new QLabel();
    label_roll = new QLabel();
    label_yaw = new QLabel();
    label_throttle = new QLabel();

    checkbox_automatic = new QCheckBox();
    checkbox_automatic->setChecked(true);

    QGridLayout* laychannel12 = new QGridLayout();
    QGridLayout* laychannel34 = new QGridLayout();

    laychannel12->addWidget(label_throttle, 0, 0);
    laychannel12->addWidget(label_yaw, 0, 1);

    laychannel34->addWidget(label_pitch, 0, 0);
    laychannel34->addWidget(label_roll, 0, 1);

    layRC->addWidget(channel12, 0, 0);
    layRC->addLayout(laychannel12, 1, 0);
    layRC->addWidget(channel34, 0, 1);
    layRC->addLayout(laychannel34, 1, 1);
    layRC->addWidget(checkbox_automatic, 2, 0);

    mainLayout->addWidget(boxRC, 0, 2, 2, 1);

    connect(checkbox_automatic, SIGNAL(stateChanged(int)), this, SLOT(on_checkbox_automatic_changed()));

}

void GUI::on_checkbox_automatic_changed()
{
    channel12->setAutomatic(checkbox_automatic->isChecked());
    channel34->setAutomatic(checkbox_automatic->isChecked());
//    share_memory->s
}

void GUI::updateThreadGUI()
{
    emit signal_updateGUI();
}

void GUI::on_updateGUI_recieved()
{
    cv::Mat image = share_memory->getImage();

    if (image.channels()==1)
        return;

    image.copyTo(this->frame);
    QImage imqt = QImage((const unsigned char*)(image.data), image.cols, image.rows, QImage::Format_RGB888);
    label_image->setPixmap(QPixmap::fromImage(imqt));

    textR_value_max->setText(QString("%1").arg(sliderR_max->value()));
    textG_value_max->setText(QString("%1").arg(sliderG_max->value()));
    textB_value_max->setText(QString("%1").arg(sliderB_max->value()));
    textR_value_min->setText(QString("%1").arg(sliderR_min->value()));
    textG_value_min->setText(QString("%1").arg(sliderG_min->value()));
    textB_value_min->setText(QString("%1").arg(sliderB_min->value()));

    share_memory->setColorFilter(sliderH_max->value(),
                                 sliderH_min->value(),
                                 sliderS_max->value(),
                                 sliderS_min->value(),
                                 sliderV_max->value(),
                                 sliderV_min->value(),
                                 sliderR_max->value(),
                                 sliderR_min->value(),
                                 sliderG_max->value(),
                                 sliderG_min->value(),
                                 sliderB_max->value(),
                                 sliderB_min->value());

    if(checkbox_automatic->isChecked()){
        channel34->setYAxis(share_memory->getAuto_Pitch());
        channel34->setXAxis(share_memory->getAuto_Roll());
        channel12->setYAxis(share_memory->getAuto_Throttle());
        channel12->setXAxis(share_memory->getAuto_Yaw());
    }

    label_throttle->setText(QString("Throttle: %1").arg(channel12->getXaxis()));
    label_yaw->setText(QString("Yaw: %1").arg(channel12->getYaxis()));

    label_roll->setText(QString("Roll: %1").arg(channel34->getYaxis()));
    label_pitch->setText(QString("Pitch: %1").arg(channel34->getXaxis()));

    share_memory->setPitch(channel34->getXaxis());
    share_memory->setRoll(channel34->getYaxis());

    share_memory->setThrottle(channel12->getXaxis());
    share_memory->setYaw(channel12->getYaxis());

    cv::Mat frame_filtered_rgb = share_memory->getImage_filtered_rgb();
    cv::Mat frame_filtered_hsv = share_memory->getImage_filtered_hsv();

    cv::Mat rgb_image_temp = share_memory->getImage_rgb();
    cv::Mat hsv_image_temp = share_memory->getImage_hsv();

    QImage imqt_rgb = QImage((const unsigned char*)(frame_filtered_rgb.data), frame_filtered_rgb.cols, frame_filtered_rgb.rows, QImage::Format_RGB888);
    label_filtered_rgb->setPixmap(QPixmap::fromImage(imqt_rgb));

    QImage imqt_hsv = QImage((const unsigned char*)(frame_filtered_hsv.data), frame_filtered_hsv.cols, frame_filtered_hsv.rows, QImage::Format_RGB888);
    label_filtered_hsv->setPixmap(QPixmap::fromImage(imqt_hsv));

    QImage imqt_rgb_temp = QImage((const unsigned char*)(rgb_image_temp.data), rgb_image_temp.cols, rgb_image_temp.rows, QImage::Format_RGB888);
    label_filtered_rgb_temp->setPixmap(QPixmap::fromImage(imqt_rgb_temp));

    QImage imqt_hsv_temp = QImage((const unsigned char*)(hsv_image_temp.data), hsv_image_temp.cols, hsv_image_temp.rows, QImage::Format_RGB888);
    label_filtered_hsv_temp->setPixmap(QPixmap::fromImage(imqt_hsv_temp));
}

void GUI::config_Image_GUI()
{
    QGroupBox* boxImage = new QGroupBox("Image");
    QGridLayout* layImage = new QGridLayout();
    layImage_original = new QGridLayout();
    boxImage->setLayout(layImage);

    label_image = new QLabel();
    label_filtered_rgb = new QLabel();
    label_filtered_hsv = new QLabel();
    label_filtered_rgb_temp = new QLabel();
    label_filtered_hsv_temp = new QLabel();
    layImage_original->addWidget(label_image, 0, 0);
    layImage->addWidget(label_filtered_rgb, 1, 0);
    layImage->addWidget(label_filtered_hsv, 1, 1);
    layImage->addWidget(label_filtered_rgb_temp, 2, 0);
    layImage->addWidget(label_filtered_hsv_temp, 2, 1);

    mainLayout->addLayout(layImage_original, 0, 0);
    mainLayout->addWidget(boxImage, 1, 0);
}

void GUI::config_color_filter()
{
    QGroupBox* boxRGB = new QGroupBox("RGB");
    QGroupBox* boxHSV = new QGroupBox("HSV");
    QGridLayout* layRGB = new QGridLayout();
    QGridLayout* layHSV = new QGridLayout();

    QGridLayout* layColorFilter = new QGridLayout();

    boxRGB->setLayout(layRGB);
    boxHSV->setLayout(layHSV);

    sliderR_min = new QSlider();
    sliderB_min = new QSlider();
    sliderG_min = new QSlider();
    sliderR_max = new QSlider();
    sliderB_max = new QSlider();
    sliderG_max = new QSlider();
    textR_min = new QLabel("R min");
    textG_min = new QLabel("G min");
    textB_min = new QLabel("B min");
    textR_max = new QLabel("R max");
    textG_max = new QLabel("G max");
    textB_max = new QLabel("B max");
    textR_value_min = new QLabel();
    textG_value_min = new QLabel();
    textB_value_min = new QLabel();
    textR_value_max = new QLabel();
    textG_value_max = new QLabel();
    textB_value_max = new QLabel();
    sliderR_min->setMaximum(255);
    sliderG_min->setMaximum(255);
    sliderB_min->setMaximum(255);
    sliderR_max->setMaximum(255);
    sliderG_max->setMaximum(255);
    sliderB_max->setMaximum(255);

    layRGB->addWidget(textR_max, 0, 0);
    layRGB->addWidget(textR_min, 0, 1);
    layRGB->addWidget(textG_max, 0, 2);
    layRGB->addWidget(textG_min, 0, 3);
    layRGB->addWidget(textB_max, 0, 4);
    layRGB->addWidget(textB_min, 0, 5);
    layRGB->addWidget(sliderR_max, 1, 0);
    layRGB->addWidget(sliderR_min, 1, 1);
    layRGB->addWidget(sliderG_max, 1, 2);
    layRGB->addWidget(sliderG_min, 1, 3);
    layRGB->addWidget(sliderB_max, 1, 4);
    layRGB->addWidget(sliderB_min, 1, 5);
    layRGB->addWidget(textR_value_max, 2, 0);
    layRGB->addWidget(textR_value_min, 2, 1);
    layRGB->addWidget(textG_value_max, 2, 2);
    layRGB->addWidget(textG_value_min, 2, 3);
    layRGB->addWidget(textB_value_max, 2, 4);
    layRGB->addWidget(textB_value_min, 2, 5);

    sliderH_min = new QDoubleSpinBox();
    sliderS_min= new QDoubleSpinBox();
    sliderV_min = new QDoubleSpinBox();
    textH_min = new QLabel("H min");
    textS_min = new QLabel("S min");
    textV_min = new QLabel("V min");
    sliderH_max = new QDoubleSpinBox();
    sliderS_max= new QDoubleSpinBox();
    sliderV_max = new QDoubleSpinBox();
    textH_max = new QLabel("H max");
    textS_max = new QLabel("S max");
    textV_max = new QLabel("V max");
    textH_value_max = new QLabel();
    textS_value_max = new QLabel();
    textV_value_max = new QLabel();
    textH_value_min = new QLabel();
    textS_value_min = new QLabel();
    textV_value_min = new QLabel();

    sliderH_min->setMaximum(6.29);
    sliderH_max->setMaximum(6.29);
    sliderS_min->setMaximum(1.1);
    sliderS_max->setMaximum(1);
    sliderV_max->setMaximum(255.1);
    sliderV_min->setMaximum(255);

    layHSV->addWidget(textH_max, 0, 0);
    layHSV->addWidget(textH_min, 0, 1);
    layHSV->addWidget(textS_max, 0, 2);
    layHSV->addWidget(textS_min, 0, 3);
    layHSV->addWidget(textV_max, 0, 4);
    layHSV->addWidget(textV_min, 0, 5);
    layHSV->addWidget(sliderH_max, 1, 0);
    layHSV->addWidget(sliderH_min, 1, 1);
    layHSV->addWidget(sliderS_max, 1, 2);
    layHSV->addWidget(sliderS_min, 1, 3);
    layHSV->addWidget(sliderV_max, 1, 4);
    layHSV->addWidget(sliderV_min, 1, 5);
    layHSV->addWidget(textH_value_max, 2, 0);
    layHSV->addWidget(textH_value_min, 2, 1);
    layHSV->addWidget(textS_value_max, 2, 2);
    layHSV->addWidget(textS_value_min, 2, 3);
    layHSV->addWidget(textV_value_max, 2, 4);
    layHSV->addWidget(textV_value_min, 2, 5);

    sliderH_max->setValue(3.27);
    sliderH_min->setValue(2.87);
    sliderS_max->setValue(0.7);
    sliderS_min->setValue(0.5);
    sliderV_max->setValue(166);
    sliderV_min->setValue(63);

    sliderR_max->setValue(0);
    sliderR_min->setValue(0);
    sliderB_max->setValue(255);
    sliderB_min->setValue(0);
    sliderG_max->setValue(255);
    sliderG_min->setValue(0);

    layColorFilter->addWidget(boxRGB, 0, 0);
    layColorFilter->addWidget(boxHSV, 1, 0);
    layImage_original->addLayout(layColorFilter, 0, 1);
    x_rgb=0;
    y_rgb=0;
    y_hsv=0;
    x_hsv=0;

    hsv_image = ColorFilter::draw_hsvmap(320);
}

void GUI::mousePressEvent(QMouseEvent *event)
{
    QPoint p = label_image->mapFromParent(QPoint(event->x(), event->y() ));

    if(p.x()>0 && p.x()<frame.cols && p.y()>0 && p.y()<frame.rows ){
        if(frame.data!=0){
            int indice = frame.step*p.y()+frame.channels()*p.x();
            int r =frame.data[indice];
            int g =frame.data[indice+1];
            int b =frame.data[indice+2];

            sliderR_max->setValue(r+20);
            sliderG_max->setValue(g+20);
            sliderB_max->setValue(b+20);

            sliderR_min->setValue(r-20);
            sliderG_min->setValue(g-20);
            sliderB_min->setValue(b-20);

            double h = ColorFilter::getH(r, g, b);
            double s = ColorFilter::getS(r, g, b);
            double v = ColorFilter::getV(r, g, b);

            sliderH_max->setValue( h*DEGTORAD + 0.2);
            sliderS_max->setValue(s + 0.1);
            sliderV_max->setValue(v+50);

            sliderH_min->setValue( h*DEGTORAD - 0.2);
            sliderS_min->setValue(s - 0.1);
            sliderV_min->setValue(v-50);
        }
    }
}


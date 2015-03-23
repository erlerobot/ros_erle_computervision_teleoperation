#include "colorfilter.h"

ColorFilter::ColorFilter()
{
}

cv::Mat ColorFilter::draw_hsvmap(int size){

    cv::Mat hsv_image;
    hsv_image.create(size, size, CV_8UC3);

    int i,j,ind;
    float x, y, H, S, scale;
    double r,g,b;
    unsigned char R,G,B;
    int grey = 175;

    for(j=0; j < size; j++){
        for(i=0; i < size; i++){
            x = (2.0*i)/size - 1.0;
            y = 1.0 - (2.0*j)/size;
            H = atan2(y,x);
            if (H>=2*PI){
                H-=2*PI;
            }
            if (H<0){
                H+=2*PI;
            }
            H = RADTODEG*H;
            S = sqrt(y*y + x*x);

            if (S<1.){
                hsv2rgb(H,S,0.7,&r,&g,&b);
                scale = 255.0;
                R = (unsigned char) (scale * r);
                G = (unsigned char) (scale * g);
                B = (unsigned char) (scale * b);
            }else{
                R=(unsigned char) grey;
                G= (unsigned char) grey;
                B= (unsigned char) grey;
            }

            ind = (size*j + i)*3;

            //printf("Valores R: %u G: %u B: %u ind: %d H: %2f S: %2f \n",R,G,B,ind,H,S);

            hsv_image.data[ind]   = R; /* Blue */
            hsv_image.data[ind+1] = G; /* Green */
            hsv_image.data[ind+2] = B; /* Red */
        }
    }
    return hsv_image;
}

void ColorFilter::hsv2rgb(double H, double S, double V, double *r, double *g, double *b)
 /* From the wikipedia: hsv color space */
  /*H entre 0 y 360, S entre 0 y 1 , V entre 0 ,255*/
{
    double h_aux,f,p,q,t, v_aux;

    h_aux = ((int)fabs(H/60.0)) % 6;
    f = (H/60.0) - h_aux;

    v_aux = V;
    p = v_aux * (1-S);
    q = v_aux * (1 - f*S);
    t = v_aux * (1 - (1-f)*S);

    if (((int)h_aux) == 0){
        *r = v_aux; *g=t; *b=p;
    }
    else if (((int)h_aux == 1)){
        *r = q; *g=v_aux; *b=p;
    }
    else if (((int)h_aux == 2)){
        *r = p; *g=v_aux; *b=t;
    }
    else if (((int)h_aux == 3)){
        *r = p; *g=q; *b=v_aux;
    }
    else if (((int)h_aux == 4)){
        *r = t; *g=p; *b=v_aux;
    }
    else if (((int)h_aux == 5)){
        *r = v_aux; *g=p; *b=q;
    }
}

double ColorFilter::getH(double r, double g, double b)
{
    double max = 0.0;
    double min = 255.0;

    if(r >= g && r >= b)
        max =  r;
    if( g >= r && g >= b )
        max =  g;
    if(b >= r && b >= g)
        max = b;

    if(r <= g && r <= b)
        min =  r;
    if( g <= r && g <= b )
        min =  g;
    if(b <= r && b <= g)
        min = b;

    if(max == min)
        return 0;

    if(max == r){
        if(g>=b){
            return (60.0*(g-b)/(max-min));
        }else{
            return ((60.0*(g-b)/(max-min))+360.0);
        }
    }
    if(max == g){
        return ((60.0*(b-r)/(max-min))+120.0);
    }
    if(max == b ){
        return ((60.0*(r-g)/(max-min))+240.0);
    }

    return 0;
}
double ColorFilter::getS(double r, double g, double b)
{
    double max = 0.0;
    double min = 255.0;

    if(r >= g && r >= b)
        max =  r;
    if( g >= r && g >= b )
        max =  g;
    if(b >= r && b >= g)
        max = b;
    if(max == 0.0)
        return 0.0;
    if(r <= g && r <= b)
        min =  r;
    if( g <= r && g <= b )
        min =  g;
    if(b <= r && b <= g)
        min = b;

    return (1.0 - (min/max)) ;
}
double ColorFilter::getV(double r, double g, double b)
{
    if(r >= g && r >= b)
        return  r;
    if( g >= r && g >= b )
        return  g;
    if(b >= r && b >= g)
        return b;

    return 0;
}

int ColorFilter::lineinimage(cv::Mat& image, int xa, int ya, int xb, int yb)
{
    cv::line(image, cv::Point(xa, ya), cv::Point(xb,yb), cv::Scalar(0, 0, 0), 1);
}

/*Draw hsv cheese functions*/
int ColorFilter::drawcircle(cv::Mat& image, int xcentro, int ycentro, int radio)
{
    cv::circle(image, cv::Point(xcentro, ycentro), radio, cv::Scalar(0, 0 ,0));
}

int ColorFilter::drawarc(cv::Mat& image, int xcentro, int ycentro, int radio, int x1, int y1, int x2, int y2)
{

    int r=0,g=0,b=0;
    int x,y;
    float i,imax;
    int SMAX=320;

    /* In this image always graf coordinates x=horizontal, y=vertical, starting at the top left corner of the image. They can't reach 240 or 320, their are not valid values for the pixels.  */
    if ((x1==x2)&&(y1==y2)){
        drawcircle(image, xcentro, ycentro, radio);
    }else{
        x1=x1-xcentro;
        y1=ycentro-y1;
        x2=x2-xcentro;
        y2=ycentro-y2;

        if (x1==0){
            if (y1<0){
            i=3*3.1416/2.;
            }else{
            i=3.1416/2.;
            }
        }else{
            if (y1==0){
                if (x1<0){
                    i=3.1416;
                }else{
                    i=0.;
                }
            }else{
                if (x1>0){
                    i=atan((float)y1/(float)x1);
                }else{
                    i=atan((float)y1/(float)x1)+3.1416;
                }
            }
        }

        i=i*RADTODEG;

        if (x2==0){
            if (y2<0){
            imax=3*3.1416/2.;
            }else{
            imax=3.1416/2.;
            }
        }else{
            if (y2==0){
                if (x2<0){
                    imax=3.1416;
                }else{
                    imax=0.;
                }
            }else{
                if (x2>0){
                    imax=atan((float)y2/(float)x2);
                }else{
                    imax=atan((float)y2/(float)x2)+3.1416;
                }
            }
        }
        imax=imax*RADTODEG;
        if (imax<i){
            imax=imax+360;
        }
        for (;i<=imax;i=i+0.1){
            x=(cos(i*DEGTORAD)*radio+xcentro);
            y=(ycentro-sin(i*DEGTORAD)*radio);

            image.data[(y*SMAX+x)*3]=b;
            image.data[(y*SMAX+x)*3+1]=g;
            image.data[(y*SMAX+x)*3+2]=r;
        }
    }
    return 0;
}

void ColorFilter::filterHSV_RGB(cv::Mat& frame, cv::Mat& rgb_filtered, cv::Mat& hsv_filtered, cv::Mat& rgb_image, cv::Mat& hsv_image,
                            double hmax, double hmin, double smax, double smin, double vmax, double vmin,
                            int rmax, int rmin, int gmax, int gmin, int bmax, int bmin)
{
    for(int x = 0; x< frame.cols; x++){
        for(int y = 0; y < frame.rows; y++ ){
            int indice = y*frame.step+x*frame.channels();
            float r_aux = (float)(unsigned int)(unsigned char) frame.data[indice];
            float g_aux = (float)(unsigned int)(unsigned char) frame.data[indice+1];
            float b_aux = (float)(unsigned int)(unsigned char) frame.data[indice+2];

            double h = ColorFilter::getH(r_aux, g_aux, b_aux);
            double s = ColorFilter::getS(r_aux, g_aux, b_aux);
            double v = ColorFilter::getV(r_aux, g_aux, b_aux);

            if( hmax>=h*DEGTORAD && hmin<=h*DEGTORAD
                && smax>=s && smin<=s
                && vmax>=v && vmin<=v ){
                hsv_filtered.data[indice]   = frame.data[indice];
                hsv_filtered.data[indice+1] = frame.data[indice+1];
                hsv_filtered.data[indice+2] = frame.data[indice+2];
            }  else {
                /* Gray Scale */
                hsv_filtered.data[indice]   = 0;//(unsigned char) (v*100/255);
                hsv_filtered.data[indice+1] = 0;//(unsigned char) (v*100/255);
                hsv_filtered.data[indice+2] = 0;//(unsigned char) (v*100/255);
            }
            h = h*DEGTORAD;

            double XX, YY, r;
            double cxy = 160;
            r = s*160;
            XX = r*cos(h)+cxy;
            YY =cxy-r*sin(h);

            int xx2 = (XX>0.0)?std::floor(XX+0.5):std::ceil(XX-0.5);
            int yy2 = (YY>0.0)?std::floor(YY+0.5):std::ceil(YY-0.5);

            int indice_hsv = yy2*rgb_image.step + xx2*rgb_image.channels();
            hsv_image.data[indice_hsv]=255;
            hsv_image.data[indice_hsv+1]=255;
            hsv_image.data[indice_hsv+2]=255;
            rgb_image.data[indice_hsv]=255;
            rgb_image.data[indice_hsv+1]=255;
            rgb_image.data[indice_hsv+2]=255;

            bool condR = (r_aux <= rmax) & ( r_aux  >= rmin);
            bool condG = (g_aux <= gmax) & (g_aux  >= gmin);
            bool condB = (b_aux<= bmax) & (b_aux >= bmin);
            if(condR & condG & condB){
                rgb_filtered.data[indice]   = frame.data[indice];
                rgb_filtered.data[indice+1] = frame.data[indice+1];
                rgb_filtered.data[indice+2] = frame.data[indice+2];
                rgb_image.data[indice_hsv]=0;
                rgb_image.data[indice_hsv+1]=0;
                rgb_image.data[indice_hsv+2]=0;
            }else{
                rgb_filtered.data[indice]   = 0;
                rgb_filtered.data[indice+1] = 0;
                rgb_filtered.data[indice+2] = 0;
            }
        }
    }
}

cv::Mat ColorFilter::filterHSV(cv::Mat& frame, cv::Mat& hsv_image, double hmax, double hmin, double smax, double smin, double vmax, double vmin)
{
    cv::Mat frame_filtered;
    frame.copyTo(frame_filtered);
    for(int x = 0; x< frame.cols; x++){
        for(int y = 0; y < frame.rows; y++ ){
            int indice = y*frame.step+x*frame.channels();
            float r_aux = (float)(unsigned int)(unsigned char) frame.data[indice];
            float g_aux = (float)(unsigned int)(unsigned char) frame.data[indice+1];
            float b_aux = (float)(unsigned int)(unsigned char) frame.data[indice+2];

            double h = ColorFilter::getH(r_aux, g_aux, b_aux);
            double s = ColorFilter::getS(r_aux, g_aux, b_aux);
            double v = ColorFilter::getV(r_aux, g_aux, b_aux);

            if( hmax>=h*DEGTORAD && hmin<=h*DEGTORAD
                && smax>=s && smin<=s
                && vmax>=v && vmin<=v ){
                frame_filtered.data[indice]   = frame.data[indice];
                frame_filtered.data[indice+1] = frame.data[indice+1];
                frame_filtered.data[indice+2] = frame.data[indice+2];
            }  else {
                /* Gray Scale */
                frame_filtered.data[indice]   = 0;//(unsigned char) (v*100/255);
                frame_filtered.data[indice+1] = 0;//(unsigned char) (v*100/255);
                frame_filtered.data[indice+2] = 0;//(unsigned char) (v*100/255);
            }
            h = h*DEGTORAD;

            double XX, YY, r;
            double cxy = 160;
            r = s*160;
            XX = r*cos(h)+cxy;
            YY =cxy-r*sin(h);

            int xx2 = (XX>0.0)?std::floor(XX+0.5):std::ceil(XX-0.5);
            int yy2 = (YY>0.0)?std::floor(YY+0.5):std::ceil(YY-0.5);

            int indice_hsv = yy2*hsv_image.step + xx2*hsv_image.channels();
            hsv_image.data[indice_hsv]=255;
            hsv_image.data[indice_hsv+1]=255;
            hsv_image.data[indice_hsv+2]=255;
        }
    }
    return frame_filtered;
}

cv::Mat ColorFilter::filterRGB(cv::Mat& frame, cv::Mat& rgb_image, int rmax, int rmin, int gmax, int gmin, int bmax, int bmin)
{
    cv::Mat frame_filtered;
    frame.copyTo(frame_filtered);
    for(int x = 0; x< frame.cols; x++){
        for(int y = 0; y < frame.rows; y++ ){
            int indice = y*frame.step+x*frame.channels();

            int red   = ( int)frame.data[indice];
            int blue  = ( int)frame.data[indice+1];
            int green = ( int)frame.data[indice+2];

            bool condR = (red <= rmax) & ( red  >= rmin);
            bool condG = (blue <= gmax) & (blue  >= gmin);
            bool condB = (green<= bmax) & (green >= bmin);

            float r_aux = (float)(unsigned int)(unsigned char) frame.data[indice];
            float g_aux = (float)(unsigned int)(unsigned char) frame.data[indice+1];
            float b_aux = (float)(unsigned int)(unsigned char) frame.data[indice+2];
            double h = ColorFilter::getH(r_aux, g_aux, b_aux);
            double s = ColorFilter::getS(r_aux, g_aux, b_aux);

            h = h*DEGTORAD;

            double XX, YY, r;
            double cxy = 160;
            r = s*160;
            XX = r*cos(h)+cxy;
            YY =cxy-r*sin(h);

            int xx2 = (XX>0.0)?std::floor(XX+0.5):std::ceil(XX-0.5);
            int yy2 = (YY>0.0)?std::floor(YY+0.5):std::ceil(YY-0.5);

            int indice_hsv = yy2*rgb_image.step + xx2*rgb_image.channels();
            rgb_image.data[indice_hsv]=255;
            rgb_image.data[indice_hsv+1]=255;
            rgb_image.data[indice_hsv+2]=255;

            if(condR & condG & condB){
                frame_filtered.data[indice]   = frame.data[indice];
                frame_filtered.data[indice+1] = frame.data[indice+1];
                frame_filtered.data[indice+2] = frame.data[indice+2];
                rgb_image.data[indice_hsv]=0;
                rgb_image.data[indice_hsv+1]=0;
                rgb_image.data[indice_hsv+2]=0;
            }else{
                frame_filtered.data[indice]   = 0;
                frame_filtered.data[indice+1] = 0;
                frame_filtered.data[indice+2] = 0;
            }
        }
    }
    return frame_filtered;
}

cv::Mat ColorFilter::drawcheese(cv::Mat& hsv_image, double sliderH_max, double sliderH_min, double sliderS_max, double sliderS_min)
{
    cv::Mat hsv_image_temp;
    hsv_image.copyTo(hsv_image_temp);
    int x1,y1,x2,y2;
    double s_max = sliderS_max*160;
    double s_min = sliderS_min*160;
    int x_centro = 160;
    int y_centro = 160;

    x1=(cos(sliderH_max)*s_min+x_centro);
    y1=(y_centro-sin(sliderH_max)*s_min);
    x2=(cos(sliderH_max)*s_max+x_centro);
    y2=(y_centro-sin(sliderH_max)*s_max);

    ColorFilter::lineinimage(hsv_image_temp,x1,y1,x2,y2);

    x1=(cos(sliderH_min)*s_min+x_centro);
    y1=(y_centro-sin(sliderH_min)*s_min);
    x2=(cos(sliderH_min)*s_max+x_centro);
    y2=(y_centro-sin(sliderH_min)*s_max);

    ColorFilter::lineinimage(hsv_image_temp,x1,y1,x2,y2);

    x1=(cos(sliderH_min)*s_max+x_centro);
    y1=(y_centro-sin(sliderH_min)*s_max);
    x2=(cos(sliderH_max)*s_max+x_centro);
    y2=(y_centro-sin(sliderH_max)*s_max);

    ColorFilter::drawarc(hsv_image_temp,x_centro,y_centro,s_max,x1,y1,x2,y2);

    x1=(cos(sliderH_min)*s_min+x_centro);
    y1=(y_centro-sin(sliderH_min)*s_min);
    x2=(cos(sliderH_max)*s_min+x_centro);
    y2=(y_centro-sin(sliderH_max)*s_min);

    ColorFilter::drawarc(hsv_image_temp,x_centro,y_centro,s_min,x1,y1,x2,y2);

    return hsv_image_temp;
}

int ColorFilter::blobDetection(cv::Mat &frame_filtered, Kalman2D *kalman, float &x, float &y)
{
    int num_pixels = 0;

    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    int thresh = 50;
    cv::Mat threshold_output;

    cv::Mat frame_filtered_gray;
    cv::cvtColor(frame_filtered, frame_filtered_gray, CV_RGB2GRAY);
    cv::threshold( frame_filtered_gray, threshold_output, thresh, 255, cv::THRESH_BINARY );
    int erosion_size = 6;
    cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS,
                  cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
                  cv::Point(erosion_size, erosion_size) );
    cv::dilate(threshold_output, threshold_output, element);
    cv::findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

    /// Approximate contours to polygons + get bounding rects and circles
    std::vector<std::vector<cv::Point> > contours_poly( contours.size() );
    std::vector<cv::Rect> boundRect;
    std::vector<cv::Point2f>center( 1 );
    std::vector<float>radius( 1 );

    for( int i = 0; i < contours.size(); i++ ){
        cv::approxPolyDP( cv::Mat(contours[i]), contours_poly[i], 3, true );
    }

    int maximo = 0;
    int indice_maximo = 0;
    for(int i= 0; i < contours_poly.size(); i++ ){
        double area0 = cv::contourArea(contours_poly[i]);
        if(area0>maximo){
            maximo = area0;
            indice_maximo = i;
        }
    }

    int dt = 1;
    if(contours_poly.size()>0){
        boundRect.push_back(boundingRect( cv::Mat(contours_poly[indice_maximo]) ) );
        cv::minEnclosingCircle( (cv::Mat)contours_poly[indice_maximo], center[0], radius[0] );

        cv::Mat roi = frame_filtered_gray(boundingRect( cv::Mat(contours_poly[indice_maximo]) ));
        num_pixels = cv::countNonZero(roi);


        float x_act = boundRect[0].x +  boundRect[0].width/2;
        float y_act = boundRect[0].y +  boundRect[0].height/2;

        float vx = (x_act-x)/dt;
        float vy = (y_act-y)/dt;

        kalman->predict(x_act, y_act, vx, vy);
        kalman->correct();

        x = x_act;
        y = y_act;
    }

    /// Draw polygonal contour + bonding rects + circles
    for( int i = 0; i< boundRect.size(); i++ ){
           cv::Scalar color = cv::Scalar( 255, 0, 0 );
//           cv::drawContours( frame_filtered, contours_poly, i, color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point() );
           cv::rectangle( frame_filtered, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
           cv::circle( frame_filtered, center[i], (int)radius[i], color, 2, 8, 0 );
    }
    cv::rectangle( frame_filtered, cv::Rect(kalman->get_x_predict()-33/2, kalman->get_y_predict()-33/2, 33, 35),
                   cv::Scalar(0, 0, 255), 2, 8, 0 );
    return num_pixels;
}



#ifndef KALMAN2D_H
#define KALMAN2D_H

#include <Eigen/Dense>

class Kalman2D
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Kalman2D();
    void predict(float x, float y , float vx, float vy);
    void correct();

    float get_x_predict();
    float get_y_predict();


private:
    Eigen::Matrix4f Q ;
    Eigen::Matrix4f P;
    Eigen::Matrix4f A ;
    Eigen::Matrix4f H ;
    Eigen::Matrix4f R ;
    Eigen::Vector4f v;

    Eigen::Vector4f x_apriori;
    Eigen::Vector4f z;
    Eigen::Matrix4f P_apriori;

    float x;
    float y;
    float vx;
    float vy;
    float dt;

};

#endif // KALMAN2D_H

#include "kalman2d.h"

Kalman2D::Kalman2D()
{
    Q = Eigen::Matrix4f::Identity()*0.0001;
    P = Eigen::Matrix4f::Identity()*100;
    A = Eigen::Matrix4f::Identity();
    H = Eigen::Matrix4f::Identity();
    R = Eigen::Matrix4f::Ones()*0.01;
    A(0, 2) = 1;
    A(1, 3) = 1;

/*
    std::cout << "Q: "<< Q << std::endl;
    std::cout << "P: "<< P << std::endl;
    std::cout << "A: "<< A << std::endl;
    std::cout << "H: "<< H << std::endl;
    std::cout << "R: "<< R << std::endl;
*/
    x=100;
    y= 100;
    vx = 0;
    vy = 0;
    dt = 0;

    v << x, y, vx, vy;
}

void Kalman2D::predict(float x, float y , float vx, float vy)
{
    x_apriori = A*v;
    z << x, y, vx, vy;
    P_apriori = A*P*A.transpose() + Q ;

}

void Kalman2D::correct()
{
    //correcion
    Eigen::Matrix4f K = (P_apriori*H.transpose()) *(H*P_apriori*H.transpose() + R).inverse();
    v = x_apriori + K*(z - H*x_apriori);

    P = (Eigen::Matrix4f::Identity() - K*H)*P_apriori;
}

float Kalman2D::get_x_predict()
{
    return v(0);
}

float Kalman2D::get_y_predict()
{
    return v(1);
}


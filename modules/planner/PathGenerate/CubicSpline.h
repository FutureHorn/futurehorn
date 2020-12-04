#ifndef __CUBICSPLINE_H__
#define __CUBICSPLINE_H__

#include <iostream>
#include <math.h>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <Eigen/Dense>

class CubicSpline
{
    public:
        CubicSpline();
        ~CubicSpline();

        void init(std::vector<double> x, std::vector<double> y);
        Eigen::MatrixXd calc_A(std::vector<double> h);
        Eigen::MatrixXd calc_B(std::vector<double> h);

        double calc(double t);
        double calcd(double t);
        double calcdd(double t);
        int search_index(double x);

    public:
        std::vector<double> m_x;
        std::vector<double> m_y;
        std::vector<double> m_a;
        std::vector<double> m_b;
        std::vector<double> m_c;
        std::vector<double> m_d;
    

        Eigen::MatrixXd m_A;
        Eigen::MatrixXd m_B;
        Eigen::MatrixXd m_C;

        int m_x_size;
};

class CubicSpline2D
{
public:
    CubicSpline2D();
    ~CubicSpline2D();

    void addEndline(std::vector<double> x, std::vector<double> y, std::vector<double> s, std::vector<double> yaw, std::vector<double> kappa);
    void setTrajectory(std::vector<cv::Point2d> waypoints);
    void calc_s(std::vector<double> x, std::vector<double> y);

    cv::Point2d get_position(double s);
    double get_kappa(double s);
    double get_yaw(double s);

    void clear();
    
public:
    CubicSpline spline_x;
    CubicSpline spline_y;
    

public:
    std::vector<double> m_waypoints_x;
    std::vector<double> m_waypoints_y;
    std::vector<double> m_waypoints_s;
    std::vector<double> m_waypoints_ds;

    double m_ds;

    std::vector<double> m_x;
    std::vector<double> m_y;
    std::vector<double> m_s;
    std::vector<double> m_yaw;
    std::vector<double> m_kappa;

};



#endif
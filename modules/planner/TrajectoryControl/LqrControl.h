#ifndef __LQRCONTROL_H__
#define __LQRCONTROL_H__

#include <iostream>
#include <math.h>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <Eigen/Dense>


typedef struct WayPoint
{
    double x; // 在地图上slam返回的x
    double y; // 在地图上slam返回的y
    double yaw; // 在地图上slam返回的theta
    double s;
    double d;
    double kappa; // 弧度 
    double v;
}WayPoint;

class LqrController
{
    public:
        LqrController();
        ~LqrController();

        
    // 内部计算
    public:
        void generateSpeed();
        WayPoint getRefpoint();

        void dlqr(Eigen::MatrixXf A, Eigen::MatrixXf B, Eigen::MatrixXf Q, Eigen::MatrixXf R);
        Eigen::MatrixXf solve_dare(Eigen::MatrixXf A, Eigen::MatrixXf B, Eigen::MatrixXf Q, Eigen::MatrixXf R);   


    public:
        std::vector<WayPoint> m_trajectory_points;

        WayPoint m_ref_point;

        double m_s_target; 

        double m_s_now;
        double m_d_now;

        double m_error_d_last;
        double m_error_yaw_now;
        double m_error_yaw_last;


        double m_kappa_max;
        double m_kappa_min;

        double m_dt;  // time tick[s]  规划一次的时间
        double m_L;  // Wheel base of the vehicle [m] 轮间距

        Eigen::MatrixXf m_lqr_Q;
        Eigen::MatrixXf m_lqr_R;
        Eigen::MatrixXf m_X;
        Eigen::MatrixXf m_K;

    // 外层调用
    public:
        void setConfig(double map_resolution, double v_max, double v_min, double omega_max, double omega_min);
        void setTrajectory(std::vector<cv::Point2d> points); // 设置轨迹
        void setTrajectory(std::vector<double> x, std::vector<double> y, std::vector<double> s, std::vector<double> yaw, std::vector<double> kappa);

       
        double m_map_resolution;
        double m_plan_speed_max;
        double m_plan_speed_min;
        double m_plan_omega_max;
        double m_plan_omega_min;

    // 核心交互
    public:
        void lqrTrajectoryPlanning();
        void receiveSlamPose(double x, double y, double theta);
        double getControl_v();
        double getControl_omega();
        double getControl_s();
        double getTrajectory_s_target();
        double m_slam_x_now; // 当前收到的信息
        double m_slam_y_now;
        double m_slam_theta_now;
        double m_slam_v_now;
        double m_slam_omega_now;

        void update();
        void update(double t);
        double getX();
        double getY();
        double getTheta();
        double m_sim_x;
        double m_sim_y;
        double m_sim_theta;


        double m_control_omega;
        double m_control_v_acc;
   
};


#endif
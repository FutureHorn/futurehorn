#ifndef _DOCKINGPLAN_H__
#define _DOCKINGPLAN_H__

#include <iostream>
#include <vector>
#include <queue>
#include <stack>
#include <thread>
#include <stdlib.h>

#include <random>
#include <math.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <Eigen/Dense>

#include "LqrControl.h"
#include "CubicSpline.h"

#include <cstdlib>
#include <ctime>
#include <chrono>
#include <iostream>
#define random(a,b) (rand()%(b-a)+a)

class DockingPlan
{
    public:
        DockingPlan();
        ~DockingPlan();

        void init();
        int initPath(double x1, double y1, double theta1, double x2, double y2, double theta2);
        int adjustSpeed(double x_initial, double y_initial, double theta_initial, double x_target, double y_target, double theta_target);
        int dockingPlanning(bool able_forward, bool able_reverse, bool able_left_spin, bool able_right_spin,
                            double x_start, double y_start, double theta_start,
                            double x_end, double y_end, double theta_end,
                            double robot_x_now, double robot_y_now, double robot_theta_now,
                            double& v, double& omega, std::string planType, bool isdraw);
        void calculateTracjectoryPoints();
        double getTimeNow();
        std::vector<cv::Point2d> generateWaypoint(double start_x, double start_y, double start_theta, 
            double end_x, double end_y, double end_theta);
        
        void showTrajectory();
        void showRobotPose();
        void clearCache();

    public:
        std::string m_planType;
        double m_x_initial;
        double m_y_initial;  
        double m_theta_initial;
        double m_x_target;
        double m_y_target;    
        double m_theta_target;
        double m_s_target;

        double m_robot_x_now;
        double m_robot_y_now;
        double m_robot_theta_now;
        double m_lqr_x;
        double m_lqr_y;
        double m_lqr_theta;

        double m_timestamp_now;
        double m_timestamp_last;


        bool m_hasIinitialized;
    
    public:
        std::vector<cv::Point2d> m_waypoints;
        CubicSpline2D m_cb;
        LqrController m_lqrController;

        int m_phase;
        
    public:
        int m_matrix_size;
        cv::Mat m_test;
        cv::Mat m_image_mini; // 等比例缩小图
        bool m_isdraw;

        double m_map_resolution; // 地图分辨率
        double m_robot_v_max; // m/s
        double m_robot_v_min; // m/s
        double m_robot_omega_max; // radian/s
        double m_robot_omega_min; // radian/s

        double m_start_spline_time; // s
        double m_end_spline_time; // s

};


#endif
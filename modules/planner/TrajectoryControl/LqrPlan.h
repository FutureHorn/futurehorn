#ifndef __LQRPLAN_H__
#define __LQRPLAN_H__

#include <iostream>
#include <vector>
#include <queue>
#include <stack>
#include <thread>
#include <stdlib.h>
#include <cstdlib>
#include <ctime>
#include <chrono>
#include <random>
#include <math.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <Eigen/Dense>

#include "LqrControl.h"
#include "CubicSpline.h"
#include "MapMaintainer.h"
#include "StructDefine.h"
#include "RobotController.h"

extern MapMaintainer g_mapMaintainer;
extern RobotStatus g_robot_status_now;

struct LqrParams
{
    double dist_start = 0;
    double dist_end = 0;

    double x_start;
    double y_start;
    double theta_start;

    double x_end;
    double y_end;
    double theta_end;

    std::string pathType;
};

class LqrPlan
{
    public:
        LqrPlan();
        ~LqrPlan();

    public:
        void reset(LqrParams params);
        int execute();
        void clearCache();
        void setDist_start(double dist);
        void setDist_end(double dist);
        int lqrPlanning(std::string pathType, MapPoint pose_start, MapPoint pose_end, MapPoint pose_now, double& v, double& omega);

    public:
        int initPath(MapPoint pose_start, MapPoint pose_end);
        std::vector<cv::Point2d> generateWaypoint();
        void outputTrajectory();

    public:
        bool m_isdraw;
        std::string m_pathType;
        std::vector<cv::Point2d> m_waypoints;
        CubicSpline2D m_cb;
        LqrController m_lqrController;

        int m_plan_phase;

        MapPoint m_pose_start;
        MapPoint m_pose_end;
        MapPoint m_pose_now;
        double m_dist_start;
        double m_dist_end;


        double m_plan_v_max;
        double m_plan_v_min;
        double m_plan_omega_max;
        double m_plan_omega_min;

        double m_s_target;
    
};

#endif
#ifndef __ROBOTCONTROLLER_H__
#define __ROBOTCONTROLLER_H__

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include "StructDefine.h"
#include "MapMaintainer.h"
#include "Common.h"

extern RobotStatus g_robot_status_now;
extern MapMaintainer g_mapMaintainer;

struct MoveParams
{
    double v = 0.5f;
    double omega = 0.2f;

    double x_initial = 10.0f;
    double y_initial = 10.0f;
    double theta_initial = 10.0f;
    double x_target = 10.0f;
    double y_target = 10.0f;
    double theta_target = 10.0f;

    double dist_reach_error = 0.002f;
    double theta_reach_error = 0.001 * CV_PI;
};

struct GoParams
{
    double go_distance = 1;

    double v = 0.5f;
    double omega = 0.0f;

    double x_initial = 10.0f;
    double y_initial = 10.0f;
    double theta_initial = 10.0f;
    double x_target = 10.0f;
    double y_target = 10.0f;
    double theta_target = 10.0f;

    double dist_reach_error = 0.02f;
};

struct RotateParams
{
    double rotate_angle = CV_PI;

    double v = 0.0f;
    double omega = 0.2f;

    double theta_initial = 10.0f;
    double theta_target = 10.0f;

    double theta_reach_error = 0.001 * CV_PI;
};

class RobotController
{
    public:
        RobotController();
        ~RobotController();

    public:
        int setMoveParams(MoveParams p);
        int move_to_pose();

        int setRotateParams(RotateParams r);
        int rotate(RobotStatus robot_status_now, double &v, double &omega);
        
        int setGoParams(GoParams g);
        int go(RobotStatus robot_status_now, double &v, double &omega);
    
    public:
        MoveParams m_move_params;
        GoParams m_go_params;
        RotateParams m_rotate_params;
        
};

#endif
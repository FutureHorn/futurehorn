#ifndef __ROBOTSIMULATOR_H__
#define __ROBOTSIMULATOR_H__

#include <iostream>
#include <memory>
#include <thread>
#include <mutex>
#include <vector>

#include "RobotControl.h"

typedef struct RobotPose
{
    float x;
    float y;
    float theta;
    float pan;
    float tilt;
    RobotPose()
    {
        x = 0.0f;
        y = 0.0f;
        theta = 0.0f;
        pan = 0.0f;
        tilt = 0.0f;
    }
    RobotPose(float x_, float y_, float theta_)
    {
        x = x_;
        y = y_;
        theta = theta_;
        pan = 0.0f;
        tilt = 0.0f;
    }
    void init()
    {
        x = 0.0f;
        y = 0.0f;
        theta = 0.0f;
        pan = 0.0f;
        tilt = 0.0f;
    }
}RobotPose;

class RobotSimulator
{
public:
    RobotSimulator();
    RobotSimulator(RobotControl *rc, RobotPose pose, int intervalTime);
    ~RobotSimulator();

	
	void start();
    RobotPose getRobotPose();
    RobotPose calcNextPose();
    void moveToPose(RobotPose pose);

protected:
	void run();

protected:
    std::thread m_task;
    int m_dtime;

    RobotPose m_robot_pose;
    float m_robot_v;
    float m_robot_omega;

    RobotControl *m_robotControl;
};



#endif
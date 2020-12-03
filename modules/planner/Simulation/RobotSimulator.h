#ifndef __ROBOTSIMULATOR_H__
#define __ROBOTSIMULATOR_H__

#include <iostream>
#include <memory>
#include <thread>
#include <mutex>
#include <vector>

#include "OdomSimulator.h"
#include "StructDefine.h"


class RobotSimulator
{
public:
    RobotSimulator();
    RobotSimulator(OdomSimulator *rc, RobotSimulatorPose pose, int intervalTime);
    ~RobotSimulator();

	
	void start();
    RobotSimulatorPose getRobotPose();
    RobotSimulatorPose calcNextPose();
    void moveToPose(RobotSimulatorPose pose);

protected:
	void run();

protected:
    std::thread m_task;
    int m_dtime;

    RobotSimulatorPose m_robot_pose;
    float m_robot_v;
    float m_robot_omega;

    OdomSimulator *m_robotControl;
};



#endif
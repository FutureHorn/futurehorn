#ifndef __STRUCTDEFINE_H__
#define __STRUCTDEFINE_H__


enum TASKTYPE
{
    TASK_MOVE,
    TASK_CHARGE
};

struct RobotStatus
{
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;

    double x_map = 0.0;
    double y_map = 0.0;
    double theta_map = 0.0;

    double battery_value = 100.0;

    double v_max = 0.3;
    double v_min = 0.2;
    double omega_max = 0.2;
    double omega_min = 0.1;

    double v = 0.0;
    double omega = 0.0;
};


struct RobotSimulatorPose
{
    float x;
    float y;
    float theta;
    float pan;
    float tilt;
    RobotSimulatorPose()
    {
        x = 0.0f;
        y = 0.0f;
        theta = 0.0f;
        pan = 0.0f;
        tilt = 0.0f;
    }
    RobotSimulatorPose(float x_, float y_, float theta_)
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
};



#endif
#ifndef __STRUCTDEFINE_H__
#define __STRUCTDEFINE_H__


enum TASKTYPE
{
    TASK_MOVE,
    TASK_CHARGE
};

struct RobotStatus
{
    double x = 0.0f;
    double y = 0.0f;
    double theta = 0.0f;

    double x_map = 0.0f;
    double y_map = 0.0f;
    double theta_map = 0.0f;

    double battery_value = 100.0f;

    double v_max = 0.3f;
    double v_min = 0.2f;
    double omega_max = 0.2f;
    double omega_min = 0.1f;
};



#endif
#ifndef __ROBOTBRAIN_H__
#define __ROBOTBRAIN_H__

#include <iostream>

#include "TaskMaintainer.h"
#include "StructDefine.h"

#define __SIMULATION__
#ifdef __SIMULATION__
    #include "LqrPlan.h"
    extern TaskMaintainer_offline g_taskMaintainer_offline;
    extern LqrPlan g_lqr_planner;
#endif


class RobotBrain
{
    public:
        RobotBrain();
        ~RobotBrain();
        int reset();
        int run();

    public:
        int UpdateTask();

    public:
        int m_flag;

        int m_tasks_num; // numbers of all task
        int m_tasks_now_id; // current executing id in all task
        int m_subtasks_num; // numbers of subtask in current task
        int m_subtasks_now_id; // current executing id in current subtask
        

        Task m_task_now;

    public:
        double m_current_x;
        double m_current_y;
        double m_current_theta;

        int m_current_x_map;
        int m_current_y_map;
        int m_current_theta_map;

        double m_able_forward;
        double m_able_back;
        double m_able_left_rotate;
        double m_able_right_rotate;
};

#endif
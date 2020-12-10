#include "RobotBrain.h"

RobotBrain::RobotBrain()
{

}

RobotBrain::~RobotBrain()
{

}

int RobotBrain::reset()
{
    m_flag = 1;
    UpdateTask();
    return 0;
}

int RobotBrain::run()
{
    if (m_task_now.type == TASK_MOVE)
    {
        if (m_subtasks_now_id == 0)
        {
            LqrParams lqr_params;
            lqr_params.pathType = "whole_path_line";
            lqr_params.dist_end = 200;
            lqr_params.x_start = g_robot_status_now.x_map;
            lqr_params.y_start = g_robot_status_now.y_map;
            lqr_params.theta_start = g_robot_status_now.theta_map;
            lqr_params.x_end = m_task_now.x_target;
            lqr_params.y_end = m_task_now.y_target;
            lqr_params.theta_end = m_task_now.theta_target;
            m_flag = g_lqr_planner.reset(lqr_params);
            
            m_subtasks_now_id += 1;
        }
        if (m_subtasks_now_id == 1)
        {
            m_flag = g_lqr_planner.execute();
        }
           
    }

    if (m_flag == 0)
    {
        m_flag = 1;
        if (m_subtasks_now_id < m_subtasks_num - 1)
        {
            m_subtasks_now_id += 1;
        }
        else
        {
            UpdateTask();
        }
    }


    return m_flag;
}

int RobotBrain::UpdateTask()
{
    
    if (g_taskMaintainer_offline.m_tasks_current_id < g_taskMaintainer_offline.m_tasks_num)
    {
        m_task_now = g_taskMaintainer_offline.getNextTask();
        std::cout<<"Update task target: "<<m_task_now.x_target<<" "<<m_task_now.y_target<<" "
                <<m_task_now.theta_target<<std::endl;
        
        m_tasks_num = g_taskMaintainer_offline.m_tasks_num;
        m_tasks_now_id = g_taskMaintainer_offline.m_tasks_current_id;
        m_subtasks_num = g_taskMaintainer_offline.m_subtasks_num;
        m_subtasks_now_id = 0;
        
        return 0;
    }
    // All tasks finished
    else
    {
        return 0;
    }
}
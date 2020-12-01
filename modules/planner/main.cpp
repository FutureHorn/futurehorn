#include <iostream>

#include "MapMaintainer.h"
#include "TaskMaintainer.h"
#include "ConfigReader.h"
#include "Drawer.h"
#include "RobotController.h"


#define __SIMULATION__

const std::string HOME_DIR = getenv("HOME");

MapMaintainer g_mapMaintainer;
TaskMaintainer_offline g_taskMaintainer_offline;
Drawer g_drawer;
ConfigReader g_configer;

RobotStatus g_robotStatus_now;



#ifdef __SIMULATION__
    #include "RobotSimulator.h"
    #include "OdomSimulator.h"
    #include "LqrControl.h"
    #include "DockingPlan.h"
#endif

RobotController g_robotController;

double m_ICPdocking_start_x;
double m_ICPdocking_start_y;
double m_ICPdocking_start_theta;
double m_ICPdocking_end_x;
double m_ICPdocking_end_y;
double m_ICPdocking_end_theta;
DockingPlan m_ICPdocking_dp;

int m_ICPdocking_phase;

double v_command;
double omega_command;

/*
int _execute_ICP_move(int x_now, int y_now, double theta_now)
{
    bool able_forward = true, able_reverse = true, able_left_spin = true, able_right_spin = true;
  
    float _v = 0.0f, _omega = 0.0f;
   
    int _flag;
    if (m_ICPdocking_phase == 0)
    {
        double dist_before_end = 1;
        double docking_x_end_before = m_ICPdocking_end_x + dist_before_end*cos(CV_PI+m_ICPdocking_end_theta);
        double docking_y_end_before = m_ICPdocking_end_y + dist_before_end*sin(CV_PI+m_ICPdocking_end_theta);
       
        m_ICPdocking_param_spin.m_target_theta = atan2(docking_y_end_before - m_ICPdocking_start_y, docking_x_end_before - m_ICPdocking_start_x);

        m_ICPdocking_phase = 1;
    }
    if (m_ICPdocking_phase == 1)
    {
        _flag = g_robotController.ExcuteSpin(able_left_spin, able_right_spin, m_ICPdocking_param_spin, robot_info_param.theta, true,
                                    _v, _omega);
        if (_flag == 0)
        {
            m_ICPdocking_phase = 2;
        }
        if (_flag == -1)
        {
            std::cout<<"docking spin blocking!"<<std::endl;
            _flag = 1;
        }
    }
    if (m_ICPdocking_phase == 2)
    {
        _flag = m_ICPdocking_dp.dockingPlanning(able_forward, able_reverse, able_left_spin, able_right_spin,
                                m_ICPdocking_start_x, m_ICPdocking_start_y, m_ICPdocking_start_theta, 
                                m_ICPdocking_end_x, m_ICPdocking_end_y, m_ICPdocking_end_theta,
                                x_now, y_now, theta_now,
                                _v, _omega, "line", true);
        if (_flag == 0)
        {
            m_ICPdocking_phase = 3;
        }
        if (_flag == -1)
        {
            std::cout<<"docking spin blocking!"<<std::endl;
            _flag = 1;
        }
        if (_flag == 2)
        {
            std::cout<<"docking plan failed!"<<std::endl;
            m_ICPdocking_phase = 0;
            _flag = 0;
        }   
    }
    if (m_ICPdocking_phase == 3)
    {
        m_ICPdocking_param_spin.m_target_theta = m_ICPdocking_end_theta;
        m_ICPdocking_phase = 4;
        m_ICPdocking_dp.init();
    }
    if (m_ICPdocking_phase == 4)
    {
        _flag = g_robotController.rotate(able_left_spin, able_right_spin, m_ICPdocking_param_spin, theta_now, true,
                            _v, _omega);
        if (_flag == 0)
        {
            m_ICPdocking_start_x = x_now;
            m_ICPdocking_start_y = y_now;
            m_ICPdocking_start_theta = theta_now;
            m_ICPdocking_phase = 5;
        }
        if (_flag == -1)
        {
            std::cout<<"docking spin blocking!"<<std::endl;
            _flag = 1;
        }
    }
    if (m_ICPdocking_phase == 5)
    {
        _flag = m_ICPdocking_dp.dockingPlanning(able_forward, able_reverse, able_left_spin, able_right_spin,
                                m_ICPdocking_start_x, m_ICPdocking_start_y, m_ICPdocking_start_theta, 
                                m_ICPdocking_end_x, m_ICPdocking_end_y, m_ICPdocking_end_theta,
                                x_now, y_now, theta_now,
                                _v, _omega, "end", true);
        if (_flag == 0)
        {
            m_ICPdocking_phase = 0;
        }
        if (_flag == -1)
        {
            std::cout<<"docking spin blocking!"<<std::endl;
            _flag = 1;
        }
        if (_flag == 2)
        {
            std::cout<<"docking plan failed!"<<std::endl;
            m_ICPdocking_phase = 0;
            _flag = 0;
        }   
    }

    v_command = _v;
    omega_command = _omega;
    
    return _flag;
}
*/


int main(int argc, char** argv)
{
    if (g_configer.readRobotInfo())
    {
        std::cout<<"robot init ok"<<std::endl;
    }
    if (g_configer.readMapInfo())
    {
        std::cout<<"map init ok"<<std::endl;
    }
    if (g_configer.readTask_offline())
    {
        std::cout<<"task init ok"<<std::endl;
    }


    g_mapMaintainer.setMap(g_configer.m_task_map_path);
    g_mapMaintainer.start();
    
    g_taskMaintainer_offline.setTask(g_configer.m_task_type);

    g_robotStatus_now.x_map = 600;
    g_robotStatus_now.y_map = 800;
    g_robotStatus_now.theta_map = CV_PI/3;


    int x_initial = 100;
    int y_initial = 200;
    double theta_initial = CV_PI/3;
    int x_target = 500;
    int y_target = 700;
    double theta_target = CV_PI/2;

#ifdef __SIMULATION__
    RobotPose simulatePG(x_initial, y_initial, theta_initial);
    OdomSimulator *odomSimulator = new OdomSimulator();
    odomSimulator->start();
    RobotSimulator *robotSimulator = new RobotSimulator(odomSimulator, simulatePG, 10);
    robotSimulator->start();
#endif

    
    bool forward = true;
    m_ICPdocking_start_x = x_initial;
    m_ICPdocking_start_y = y_initial;
    if (forward)
    {
        m_ICPdocking_start_theta = theta_initial;
    }
    else
    {
        m_ICPdocking_start_theta = theta_initial - CV_PI;
    }
    m_ICPdocking_end_x = x_target; 
    m_ICPdocking_end_y = y_target;
    m_ICPdocking_end_theta = theta_target;
    m_ICPdocking_dp.init();
    m_ICPdocking_phase = 0;


    std::cout<<"planner init success!"<<std::endl;
    
    
    
    
    while(true)
    {
        g_drawer.draw();
    }
    


    return 1;
}
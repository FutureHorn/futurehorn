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

RobotController g_robot_controller;
bool g_robot_controller_ready;
RotateParams g_robot_rotate_params;
GoParams g_robot_go_params;
MoveParams g_robot_move_params;

RobotStatus g_robot_status_now;



#ifdef __SIMULATION__
    #include "RobotSimulator.h"
    #include "OdomSimulator.h"
    #include "LqrControl.h"
    #include "DockingPlan.h"
#endif

#ifdef __SIMULATION__
    RobotSimulatorPose g_robotSimulator_current_pose;
#endif

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


int _execute_ICP_move()
{
    bool able_forward = true, able_reverse = true, able_left_spin = true, able_right_spin = true;
  
    double _v = 0.0f, _omega = 0.0f;
   
    int _flag;
    if (m_ICPdocking_phase == 0)
    {
        double dist_before_end = 1;
        double docking_x_end_before = m_ICPdocking_end_x + dist_before_end*cos(CV_PI+m_ICPdocking_end_theta);
        double docking_y_end_before = m_ICPdocking_end_y + dist_before_end*sin(CV_PI+m_ICPdocking_end_theta);
       
        g_robot_rotate_params.theta_target = atan2(docking_y_end_before - m_ICPdocking_start_y, docking_x_end_before - m_ICPdocking_start_x);

        m_ICPdocking_phase = 1;
    }
    if (m_ICPdocking_phase == 1)
    {
        _flag = _flag = g_robot_controller.rotate(g_robot_status_now, _v, _omega);
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
                                g_robot_status_now.x_map, g_robot_status_now.y_map, g_robot_status_now.theta_map,
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
        g_robot_rotate_params.theta_target = m_ICPdocking_end_theta;
        m_ICPdocking_phase = 4;
        m_ICPdocking_dp.init();
    }
    if (m_ICPdocking_phase == 4)
    {
        _flag = g_robot_controller.rotate(g_robot_status_now, _v, _omega);
        if (_flag == 0)
        {
            m_ICPdocking_start_x = g_robot_status_now.x_map;
            m_ICPdocking_start_y = g_robot_status_now.y_map;
            m_ICPdocking_start_theta = g_robot_status_now.theta_map;
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
                                g_robot_status_now.x_map, g_robot_status_now.y_map, g_robot_status_now.theta_map,
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

    g_robot_status_now.x_map = 800;
    g_robot_status_now.y_map = 800;
    g_robot_status_now.theta_map = CV_PI/3;

#ifdef __SIMULATION__
    RobotSimulatorPose g_robotSimulator_current_pose(g_robot_status_now.x_map, g_robot_status_now.y_map, g_robot_status_now.theta_map);
    OdomSimulator *odomSimulator = new OdomSimulator();
    odomSimulator->start();
    RobotSimulator *robotSimulator = new RobotSimulator(odomSimulator, g_robotSimulator_current_pose, 10);
    robotSimulator->start();
#endif


    int x_target = 1500;
    int y_target = 3700;
    double theta_target = CV_PI/2;


    
    bool forward = true;
    m_ICPdocking_start_x = g_robot_status_now.x_map;
    m_ICPdocking_start_y = g_robot_status_now.y_map;
    if (forward)
    {
        m_ICPdocking_start_theta = g_robot_status_now.theta_map;
    }
    else
    {
        m_ICPdocking_start_theta = g_robot_status_now.theta_map - CV_PI;
    }
    m_ICPdocking_end_x = x_target; 
    m_ICPdocking_end_y = y_target;
    m_ICPdocking_end_theta = theta_target;
    m_ICPdocking_dp.init();
    m_ICPdocking_phase = 0;


    std::cout<<"planner init success!"<<std::endl;
    
    
    
    int task_flag = 1;
    int task_phase = 0;
    while (task_flag == 1)
    {
    
        double v, omega;
        g_robotSimulator_current_pose = robotSimulator->getRobotPose();
        g_robot_status_now.x_map = g_robotSimulator_current_pose.x;
        g_robot_status_now.y_map = g_robotSimulator_current_pose.y;
        g_robot_status_now.theta_map = g_robotSimulator_current_pose.theta;

        if (task_phase == 0)
        {
            // if (!g_robot_controller_ready)
            // {
            //     g_robot_rotate_params.theta_target = g_robot_status_now.theta_map - CV_PI/4;
            //     NormalizeAngle(g_robot_rotate_params.theta_target);
            //     g_robot_controller.setRotateParams(g_robot_rotate_params);
            //     g_robot_controller_ready = true;
            // }
            // task_flag = g_robot_controller.rotate(g_robot_status_now, v, omega);
            task_flag = _execute_ICP_move();std::cout<<"!!!"<<std::endl;
        }
        
        if (task_phase == 1)
        {
            if (!g_robot_controller_ready)
            {
                g_robot_go_params.go_distance = -100;

                g_robot_controller.setGoParams(g_robot_go_params);
                g_robot_controller_ready = true;
            }
            task_flag = g_robot_controller.go(g_robot_status_now, v, omega);
        }

        

        odomSimulator->sendToOdometry(v, omega);
        
        // std::cout<<g_robotStatus_now.theta_map<<std::endl;
        g_robot_status_now.v = v;
        g_robot_status_now.omega = omega;

        if (task_flag == 0)
        {
            if (task_phase == 0)
            {
                task_phase = 1;
                task_flag = 1;
                g_robot_controller_ready = false;
            }
            else
            {
                break;
            }
          
        }
        

        g_drawer.draw();
    }
    


    return 1;
}
#include <iostream>

#include "MapMaintainer.h"
#include "TaskMaintainer.h"
#include "ConfigReader.h"
#include "Drawer.h"
#include "RobotController.h"
#include "RobotBrain.h"


#define __SIMULATION__

const std::string HOME_DIR = getenv("HOME");

MapMaintainer g_mapMaintainer;
TaskMaintainer_offline g_taskMaintainer_offline;
Drawer g_drawer;
ConfigReader g_configer;
RobotBrain g_robotBrain;

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
    #include "LqrPlan.h"
#endif

#ifdef __SIMULATION__
    RobotSimulatorPose g_robotSimulator_current_pose;

    LqrPlan g_lqr_planner;
#endif



double g_command_v;
double g_command_omega;

MapPoint pose_start, pose_end;


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
  


    g_mapMaintainer.setMap(g_configer.m_task_map_path);
    g_mapMaintainer.start();
   
    g_taskMaintainer_offline.loadFile();

    g_robot_status_now.x_map = 700;
    g_robot_status_now.y_map = 800;
    g_robot_status_now.theta_map = CV_PI/3;

#ifdef __SIMULATION__
    RobotSimulatorPose g_robotSimulator_current_pose(g_robot_status_now.x_map, g_robot_status_now.y_map, g_robot_status_now.theta_map);
    OdomSimulator *odomSimulator = new OdomSimulator();
    odomSimulator->start();
    RobotSimulator *robotSimulator = new RobotSimulator(odomSimulator, g_robotSimulator_current_pose, 10);
    robotSimulator->start();
#endif

    std::cout<<"planner init success!"<<std::endl;
     
    
    int task_flag = 1;
    int task_phase = 0;
    while (task_flag != 0)
    {

        g_robotSimulator_current_pose = robotSimulator->getRobotPose();
        g_robot_status_now.x_map = g_robotSimulator_current_pose.x;
        g_robot_status_now.y_map = g_robotSimulator_current_pose.y;
        g_robot_status_now.theta_map = g_robotSimulator_current_pose.theta;

        task_flag = g_robotBrain.run();
        
        // if (task_phase == 0)
        // {
        //     // if (!g_robot_controller_ready)
        //     // {
        //     //     g_robot_rotate_params.theta_target = g_robot_status_now.theta_map - CV_PI/4;
        //     //     NormalizeAngle(g_robot_rotate_params.theta_target);
        //     //     g_robot_controller.setRotateParams(g_robot_rotate_params);
        //     //     g_robot_controller_ready = true;
        //     // }
        //     // task_flag = g_robot_controller.rotate(g_robot_status_now, g_command_v, g_command_omega);
        //     LqrParams lqr_params;
        //     lqr_params.pathType = "whole_path_line";
        //     lqr_params.dist_end = 200;
        //     lqr_params.x_start = g_robot_status_now.x_map;
        //     lqr_params.y_start = g_robot_status_now.y_map;
        //     lqr_params.theta_start = g_robot_status_now.theta_map;
        //     lqr_params.x_end = 1100;
        //     lqr_params.y_end = 1100;
        //     lqr_params.theta_end = CV_PI/3;
        //     task_flag = g_lqr_planner.reset(lqr_params);
        // }
        // if (task_phase == 1)
        // {
        //     task_flag = g_lqr_planner.execute();
        // }
        
        // if (task_phase == 2)
        // {
        //     // if (!g_robot_controller_ready)
        //     // {
        //     //     g_robot_go_params.go_distance = -100;

        //     //     g_robot_controller.setGoParams(g_robot_go_params);
        //     //     g_robot_controller_ready = true;
        //     // }
        //     // task_flag = g_robot_controller.go(g_robot_status_now, g_command_v, g_command_omega);
        //     task_flag = 0;
        // }

        

        odomSimulator->sendToOdometry(g_command_v, g_command_omega);
        
        // std::cout<<g_robotStatus_now.theta_map<<std::endl;
        g_robot_status_now.v = g_command_v;
        g_robot_status_now.omega = g_command_omega;

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
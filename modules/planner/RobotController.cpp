#include "RobotController.h"
 
RobotController::RobotController()
{

}

RobotController::~RobotController()
{

}

int RobotController::setMoveParams(MoveParams m)
{
    m_move_params = m;
    return 0;
}

int RobotController::move_to_pose()
{
    return 0;
}

int RobotController::setGoParams(GoParams g)
{
    m_go_params = g;
    m_go_params.x_initial = g_robot_status_now.x_map;
    m_go_params.y_initial = g_robot_status_now.y_map;
    m_go_params.theta_initial = g_robot_status_now.theta_map;

    m_go_params.x_target = g_robot_status_now.x_map + m_go_params.go_distance * cos(g_robot_status_now.theta_map);
    m_go_params.y_target = g_robot_status_now.y_map + m_go_params.go_distance * sin(g_robot_status_now.theta_map);
    m_go_params.theta_target = g_robot_status_now.theta_map;
    return 0;
}

int RobotController::go(RobotStatus robot_status_now, double &v, double &omega)
{
    omega = 0;

    double gap = sqrt(pow(m_go_params.x_target - robot_status_now.x_map, 2) + pow(m_go_params.y_target - robot_status_now.y_map, 2)) 
        * g_mapMaintainer.m_map_resolution;

    std::cout<<sqrt(pow(m_go_params.x_target - robot_status_now.x_map, 2) + pow(m_go_params.y_target - robot_status_now.y_map, 2)) <<std::endl;
    std::cout<<"dist error: "<<gap<<std::endl;

    if ( fabs(gap) > fabs(m_go_params.v))
    {
        v = m_go_params.v * m_go_params.go_distance / fabs(m_go_params.go_distance) ;
    }
    else
    {
        v = 0.1 * m_go_params.go_distance / fabs(m_go_params.go_distance);
    }
    

    if ( fabs(gap) < m_go_params.dist_reach_error)
    {
        return 0;
    }
    else
    {
        return 1;
    }
}

int RobotController::setRotateParams(RotateParams r)
{
    m_rotate_params = r;
    m_rotate_params.theta_initial = g_robot_status_now.theta_map;
    return 0;
}

int RobotController::rotate(RobotStatus robot_status_now, double &v, double &omega)
{
    v = 0;

    double gap = m_rotate_params.theta_target - robot_status_now.theta_map;
    NormalizeAngle(gap);

    std::cout<<"theta error: "<<gap<<std::endl;

    if ( fabs(gap) > fabs(m_rotate_params.omega))
    {
        omega = m_rotate_params.omega * gap / fabs(gap);
    }
    else
    {
        omega = 0.1 * gap / fabs(gap);
    }
    
    if ( fabs(gap) < m_rotate_params.theta_reach_error)
    {
        return 0;
    }
    else
    {
        return 1;
    }
}


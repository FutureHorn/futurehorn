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
    return 0;
}

int RobotController::go(RobotStatus robot_status_now, double &v, double &omega)
{
    omega = 0;

    double gap = sqrt(pow(m_go_params.x_target - robot_status_now.x_map, 2) + pow(m_go_params.x_target - robot_status_now.x_map, 2));

    std::cout<<gap<<std::endl;

    if ( fabs(gap) > fabs(m_go_params.v))
    {
        v = m_go_params.v * gap / fabs(gap);
    }
    else
    {
        v = 0.01 * gap / fabs(gap);
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
    return 0;
}

int RobotController::rotate(RobotStatus robot_status_now, double &v, double &omega)
{
    v = 0;

    double gap = m_rotate_params.theta_target - robot_status_now.theta_map;
    NormalizeAngle(gap);

    std::cout<<gap<<std::endl;

    if ( fabs(gap) > fabs(m_rotate_params.omega))
    {
        omega = m_rotate_params.omega * gap / fabs(gap);
    }
    else
    {
        omega = 0.01 * gap / fabs(gap);
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


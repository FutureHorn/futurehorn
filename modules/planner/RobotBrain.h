#ifndef __ROBOTBRAIN_H__
#define __ROBOTBRAIN_H__

#include <iostream>

class RobotBrain
{
    public:
        RobotBrain();
        ~RobotBrain();
    
    public:
        int reset();

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
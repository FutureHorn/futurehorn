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
};

#endif
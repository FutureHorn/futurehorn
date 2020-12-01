#ifndef __ROBOTCONTROLLER_H__
#define __ROBOTCONTROLLER_H__

#include "StructDefine.h"

class RobotController
{
    public:
        RobotController();
        ~RobotController();

    public:
        int move_to_pose();
        int rotate(double angle_now, double angle_need, double &v, double &omega);
        int goStraight();
        int rotate_toAngle(double angle_now, double angle_target, double &v, double &omega); 

        
};

#endif
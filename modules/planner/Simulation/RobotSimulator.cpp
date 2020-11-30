#include "RobotSimulator.h"

RobotSimulator::RobotSimulator()
{
    
}

RobotSimulator::RobotSimulator(RobotControl *rc, RobotPose pose, int intervalTime)
{
    m_robotControl = rc;
    m_robot_pose = pose;
    // std::cout<<"init robot pose: "<<m_robot_pose.x<<" "<<m_robot_pose.y<<" "<<m_robot_pose.theta<<std::endl;
    m_dtime = intervalTime;
}

RobotSimulator::~RobotSimulator()
{
    
}

void RobotSimulator::start()
{
	m_robot_v = 0.0f;  // 初始化v和omega，使得机器人最开始是静止的
	m_robot_omega = 0.0f;
	m_task = (std::thread(std::bind(&RobotSimulator::run, this)));
}

void RobotSimulator::run()
{
    while (true)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(m_dtime)); // 时间间隔（毫秒）

		m_robotControl->getFromOdometry(m_robot_v, m_robot_omega); // 得到下位机实际能够提供的v和omega
       
        RobotPose next_pose = calcNextPose();

        moveToPose(next_pose);
	}
}

RobotPose RobotSimulator::getRobotPose()
{
    // std::cout<<"get pose: "<<m_robot_pose.x<<" "<<m_robot_pose.y<<" "<<m_robot_pose.theta<<std::endl;

    return m_robot_pose;
}

void RobotSimulator::moveToPose(RobotPose pose)
{
    m_robot_pose = pose;  // 机器人跳到下一个点位
}

RobotPose RobotSimulator::calcNextPose()
{
    // std::cout<<"机器人收到的速度v: "<<m_robot_v<<"角速度omega: "<<m_robot_omega<<std::endl;
    float omega, v;
    v = m_robot_v*(1.0f);
    omega = m_robot_omega*(1.0f); // v和omega乘上了一个系数，这个系数过大时发现机器人在绕圈
    
    float dtheta = omega*0.001f*m_dtime; // 时间是毫秒，角速度单位是秒
    RoundTheta(dtheta);

    float dx = cos(m_robot_pose.theta) * v * 0.001f*m_dtime; // 沿着当前小车方向，以速度v来计算出新的机器人位置
    float dy = sin(m_robot_pose.theta) * v * 0.001f*m_dtime;

    RobotPose tmp;
    tmp.x = m_robot_pose.x + dx;
    tmp.y = m_robot_pose.y + dy;
    tmp.theta = m_robot_pose.theta + dtheta;
    RoundTheta(tmp.theta);

    return tmp;
}
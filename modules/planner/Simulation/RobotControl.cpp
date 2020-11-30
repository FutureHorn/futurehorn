#include "RobotControl.h"


using namespace Eigen;

void RoundTheta(float &theta)
{
    if (theta > CV_PI)
    {
        theta -= 2 * CV_PI;
    }
    else if (theta <= -CV_PI)
    {
        theta += 2 * CV_PI;
    }
}


RobotControl::RobotControl()
{
	m_v_in = 0.0f;
    m_omega_in = 0.0f;
	m_v_out = 0.0f;
	m_omega_out = 0.0f; 
}

RobotControl::~RobotControl()
{

}

// 机器人控制自己是一个线程
bool RobotControl::start()
{
	m_isReceivedCommand = false;
	m_task = (std::thread(std::bind(&RobotControl::run, this)));
	return true;
}

// 
void RobotControl::run()
{
	while (true)
	{
		while (!m_isReceivedCommand) // 如果没有发送v和omega，系统一直sleep等待, 发送v和omega会将这个变量设置为true
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
		m_isReceivedCommand = false; // 重新设置为false状态

		simulationOdometry();
        
	}
}

void RobotControl::simulationOdometry()
{
	float tdq1 = (m_v_in - m_omega_in*D*0.5)/R; // 输入到机器人的v和omega，由于机器人的实际负载，会产生误差
	float tdq2 = (m_v_in + m_omega_in*D*0.5)/R; // 这两行是左右轮子在角速度omega下的移动弧度

	// 下面这段，实际上模拟下位机，输入命令v和omega后，返回真实的v和omega
	for (int i = 0; i < N_control_circle; i++)  // N_control_circle现在是10
	{
		float delta_q1 = tdq1 - dq1; // dq1和dq2初始化都是0
		float delta_q2 = tdq2 - dq2;
		//printf("target %f |", m_v_in, m_omega_in);
		//printf("delta %f %f |", delta_q1/motor_sigma, delta_q2/motor_sigma);

		float N1 = motor_max_torque*(1.0/(1 + exp(-delta_q1/motor_sigma)) - 0.5);
		float N2 = motor_max_torque*(1.0/(1 + exp(-delta_q2/motor_sigma)) - 0.5);

		// printf("N %f %f %f |", motor_max_torque, N1, N2);

		//m_v = m_v_in;
		//m_omega = m_omega_in;

		Matrix2d A;
		A << I1 + I*R*R/(D*D) + 0.5*m*R*R, - I*R*R/(D*D) + 0.5*m*R*R,
			- I*R*R/(D*D) + 0.5*m*R*R, I2 + I*R*R/(D*D) + 0.5*m*R*R;

		Vector2d b(N1, N2);
		Vector2d C = A.colPivHouseholderQr().solve(b);

		ddq1 = C(0);
		ddq2 = C(1);

		//printf("ddq %f %f |", ddq1, ddq2);

		dq1 = dq1 + ddq1*dt;
		dq2 = dq2 + ddq2*dt;

		//printf("dq %f %f |", dq1, dq2);

		m_v_out = 0.5*(dq1*R + dq2*R);
		m_omega_out = (dq2*R - dq1*R)/D;
		// printf("v omega %f %f\n", v, omega);
	}
}

// 上位机发送命令: v和omega
void RobotControl::sendToOdometry(float v_in, float omega_in)
{
    m_isReceivedCommand = true;
    m_v_in = v_in * 0.001;  // 下位机接受到的是像素位置/s，计算时单位需要转成m/s
    m_omega_in = -omega_in; // 下位机的omega方向与像素方向相反
	// std::cout<<"下位机接受到的命令 v: "<<m_v_in<<" omega:"<<omega_in<<std::endl;
}

// 下位机返回实际的值:v和omega
void RobotControl::getFromOdometry(float &v_out, float &omega_out)
{
	v_out = m_v_out*1000; // 下位机计算时单位是m/s,传给机器人模拟器的单位是像素/s  
	omega_out = -m_omega_out; // 下位机的omega方向与像素方向相反
	// std::cout<<"下位机输出到机器人的 v: "<<v_out<<" omega:"<<omega_out<<std::endl;
}
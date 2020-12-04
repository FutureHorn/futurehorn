#ifndef __ODOMSIMULATOR_H__
#define __ODOMSIMULATOR_H__

#include <iostream>
#include <memory>
#include <thread>
#include <mutex>
#include <vector>
#include <Eigen/Dense>
#include <chrono>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "MapMaintainer.h"

extern MapMaintainer g_mapMaintainer;

void RoundTheta(float &theta);

class OdomSimulator
{
public:
	OdomSimulator();
	~OdomSimulator();

	bool start();
    void simulationOdometry();
    void sendToOdometry(float v_in, float omega_in);
	void getFromOdometry(float &v_out, float &omega_out);

protected:
	void run();
	
	std::thread m_task;

	float m_v_in;  // 上位机发送命令的v和omega
	float m_omega_in;
	float m_v_out; // 下位机返回实际的v和omega
	float m_omega_out;

    bool m_isReceivedCommand;

    // parameter of odometry
    float dq1 = 0.0f;
    float dq2 = 0.0f;
    float ddq1 = 0.0f;
    float ddq2 = 0.0f;

    float m = 1.0f; //scalable, only I/m, I1/m, motor_max_torque/m matters
    float D = 1.0f; //m, this must has dimension as m, so the result can be m/s
    float R = D/5.0f;
    float I = m*D*D/8.0;
    float I1 = I/10.0f;
    float I2 = I/10.0f;

    float motor_sigma = 5;
    float motor_max_torque = 100*(I1 + I*R*R/(D*D) + 0.5*m*R*R); // 模拟力矩

    float N_control_circle = 10;
    float dt = 0.02/N_control_circle; //s, this must has dimension as s, so the result can be m/s

};

#endif
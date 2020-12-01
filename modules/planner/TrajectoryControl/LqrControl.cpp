#include "LqrControl.h"

// Angle normalization to [-pi,pi]
double RoundTheta(double angle) {
    double a = fmod(angle + M_PI, 2.0 * M_PI);
    if (a < 0.0) 
    {
        a += (2.0 * M_PI);
    }
    return a - M_PI;
}

LqrController::LqrController()
{
    m_dt = 0.01;  // time tick[s]  规划一次的时间
    m_L = 0.5;  // Wheel base of the vehicle [m] 轮间距

    m_error_d_last = 0;
    m_error_yaw_last = 0;

    m_lqr_Q = Eigen::MatrixXf::Identity(5,5);
    m_lqr_R = Eigen::MatrixXf::Identity(2,2);
}

LqrController::~LqrController()
{
    
}

void LqrController::setConfig(double map_resolution, double v_max, double v_min, double omega_max, double omega_min)
{
    m_map_resolution = map_resolution;
    m_plan_speed_max = v_max / map_resolution;
    m_plan_speed_min = v_min / map_resolution;
    m_plan_omega_max = omega_max;
    m_plan_omega_min = omega_min;
}


void LqrController::update()
{
     // if m_control_omega >= max_steer:
    //     delta = max_steer
    // if m_control_omega <= - max_steer:
    //     delta = - max_steer
    
    // m_sim_x = m_slam_x_now + m_ref_point.v * cos(m_slam_theta_now) * m_dt;
    // m_sim_y = m_slam_y_now + m_ref_point.v * sin(m_slam_theta_now) * m_dt;
    // m_sim_theta = m_slam_theta_now + m_ref_point.v / m_L * tan(m_control_omega) * m_dt;
    // state.v = state.v + a * dt;
    
    // std::cout<<"a:"<<m_slam_x_now<<std::endl;
    // std::cout<<"b:"<<m_slam_y_now<<std::endl;
    // std::cout<<"c:"<<m_slam_theta_now<<std::endl;
    

    // m_sim_x = m_slam_x_now + m_ref_point.v * cos(m_slam_theta_now) * t;
    // m_sim_y = m_slam_y_now + m_ref_point.v * sin(m_slam_theta_now) * t;
    // m_sim_theta = m_slam_theta_now + m_ref_point.v / m_L * tan(m_control_omega) * t;

    // // m_sim_theta = RoundTheta(m_sim_theta);
    // std::cout<<"1:"<<m_ref_point.v * cos(m_slam_theta_now) * t<<std::endl;
    // std::cout<<"2:"<<m_ref_point.v * sin(m_slam_theta_now) * t<<std::endl;
    // std::cout<<"3:"<<m_ref_point.v / m_L * tan(m_control_omega) * t<<std::endl;

    
    m_slam_x_now = m_slam_x_now + m_ref_point.v * cos(m_slam_theta_now) * m_dt;
    m_slam_y_now = m_slam_y_now + m_ref_point.v * sin(m_slam_theta_now) * m_dt;
    m_slam_theta_now = m_slam_theta_now + m_control_omega * m_dt;
}

void LqrController::update(double t)
{
    m_slam_x_now = m_slam_x_now + m_ref_point.v * cos(m_slam_theta_now) * t;
    m_slam_y_now = m_slam_y_now + m_ref_point.v * sin(m_slam_theta_now) * t;
    m_slam_theta_now = m_slam_theta_now + m_control_omega * t;
}

double LqrController::getX()
{
    // return m_sim_x;
    return m_slam_x_now;
}

double LqrController::getY()
{
    // return m_sim_y;
    return m_slam_y_now;
}

double LqrController::getTheta()
{
    // return m_sim_theta;
    return m_slam_theta_now;
}

void LqrController::lqrTrajectoryPlanning()
{
    m_ref_point = getRefpoint();
    
    double e = m_d_now;

    double tv = m_ref_point.v; // 取参考点的速度

    double k = m_ref_point.kappa; // 取参考点的弧度
    m_slam_v_now = m_ref_point.v; 
    double v = m_slam_v_now; // 当前速度
    m_error_yaw_now = RoundTheta(m_slam_theta_now - m_ref_point.yaw); // 当前的偏航，和参考点偏航的差距
    double th_e = RoundTheta(m_slam_theta_now - m_ref_point.yaw);


    Eigen::MatrixXf A = Eigen::MatrixXf::Zero(5,5);
    // A = [1.0, dt, 0.0, 0.0, 0.0
    //      0.0, 0.0, v, 0.0, 0.0]
    //      0.0, 0.0, 1.0, dt, 0.0]
    //      0.0, 0.0, 0.0, 0.0, 0.0]
    //      0.0, 0.0, 0.0, 0.0, 1.0]
    A(0,0) = 1.0;
    A(0,1) = m_dt;
    A(1,2) = v;
    A(2,2) = 1.0;
    A(2,3) = m_dt;
    A(4,4) = 1.0;

    Eigen::MatrixXf B = Eigen::MatrixXf::Zero(5,2);
    // B = [0.0, 0.0
    //     0.0, 0.0
    //     0.0, 0.0
    //     v/L, 0.0
    //     0.0, dt]
    B(3,0) = v / m_L;
    B(4,1) = m_dt;


    // A和B是根据规则构建的
    dlqr(A, B, m_lqr_Q, m_lqr_R);


    // state vector
    // x = [e, dot_e, th_e, dot_th_e, delta_v]
    // e: lateral distance to the path  frenet横向距离
    // dot_e: derivative of e    e的导数
    // th_e: angle difference to the path   偏航角度
    // dot_th_e: derivative of th_e  偏航角度的导数
    // delta_v: difference between current speed and target speed  当前速度与目标速度的差值
    Eigen::MatrixXf x = Eigen::MatrixXf::Zero(5,1);
    x(0,0) = e; // e
    x(1,0) = (e - m_error_d_last) / m_dt; // e - pe
    x(2,0) = th_e; // th_e
    x(3,0) = (th_e - m_error_yaw_last) / m_dt; // (th_e - pth_e)
    x(4,0) = v - tv; // v - tv

    // input vector
    // u = [delta, accel]
    // delta: steering angle  转向角速度
    // accel: acceleration   转向角加速度
    Eigen::MatrixXf ustar = -m_K * x;

    // calc steering input
    double ff = atan2(m_L * m_ref_point.kappa, 1);  // feedforward steering angle  前馈转向角
    double fb = RoundTheta(ustar(0,0));  // feedback steering angle   反馈转角
    
    m_control_omega = ff + fb;

    // calc accel input
    m_control_v_acc = ustar(1,0);

    m_error_d_last = m_d_now;
    m_error_yaw_last = m_error_yaw_now;

    if (fabs(m_control_omega) > m_plan_omega_max)
    {
        m_control_omega = m_plan_omega_max * m_control_omega/fabs(m_control_omega);
    }
    
    std::cout<<"???????????? d: "<<m_d_now * m_map_resolution<<" m"<<std::endl;
    std::cout<<"???????????? v: "<<m_ref_point.v * m_map_resolution<<" m/s"<<std::endl;
    std::cout<<"???????????? kappa: "<<m_ref_point.kappa<<std::endl;
    std::cout<<"???????????? omega: "<<m_control_omega<<std::endl;
    std::cout<<"???????????? v acc: "<<m_control_v_acc<<std::endl;
}

// 求解黎卡提方程 dy/dx=P(x)y^2+Q(x)y+R(x)
Eigen::MatrixXf LqrController::solve_dare(Eigen::MatrixXf A, Eigen::MatrixXf B, Eigen::MatrixXf Q, Eigen::MatrixXf R)
{
    //solve a discrete time_Algebraic Riccati equation (DARE)
    
    Eigen::MatrixXf x = Q;
    Eigen::MatrixXf x_next = Q;
    int max_iter = 150;  // 竟然还迭代
    double eps = 0.01; // 迭代阈值

    // TODO 不知道这是在干嘛，可以照抄
    for (int i = 0 ; i < max_iter; i++)
    {
        x_next = A.transpose() * x * A - A.transpose() * x * B * 
            (R + B.transpose() * x * B).inverse() * B.transpose() * x * A + Q;
        
        Eigen::ArrayXXf tmp = (x_next- x).array().abs();

        if (tmp.maxCoeff() < eps)
        {
            break;
        }
        x = x_next;
    }
    return x_next;
}
  

// 求解离散时间lqr控制器
// A 5*5
// B 5*2
// Q
void LqrController::dlqr(Eigen::MatrixXf A, Eigen::MatrixXf B, Eigen::MatrixXf Q, Eigen::MatrixXf R)
{
    //
    // // Solve the discrete time lqr controller.
    // // x[k+1] = A x[k] + B u[k]
    // // cost = sum x[k].T*Q*x[k] + u[k].T*R*u[k]
    // // ref Bertsekas, p.151

    // // first, try to solve the ricatti equation
    m_X = solve_dare(A, B, Q, R);

    // // compute the LQR gain
    m_K = (B.transpose() * m_X * B + R).inverse() * (B.transpose()* m_X *A);    
    
    Eigen::EigenSolver<Eigen::MatrixXf> eigen_solver( A - B * m_K );
    Eigen::MatrixXf eig_result = eigen_solver.pseudoEigenvalueMatrix();

    int rows = eig_result.rows();
    Eigen::MatrixXf new_value = Eigen::MatrixXf::Zero(rows,1);
    for (int i = 0 ; i < rows; i++)
    {
        new_value(i,0) = eig_result(i,i);
    }
 
}

WayPoint LqrController::getRefpoint()
{
    double dist_min = 1e10;
    int dist_min_id;
    for (int i = 0 ; i < m_trajectory_points.size(); i++)
    {
        double dx = m_slam_x_now - m_trajectory_points[i].x;
        double dy = m_slam_y_now - m_trajectory_points[i].y;
        double dist = sqrt(pow(dx,2)+pow(dy,2));

        if (dist < dist_min)
        {
            dist_min = dist;
            dist_min_id = i;
        }
    }
    m_d_now = dist_min;

    double dxl = m_trajectory_points[dist_min_id].x - m_slam_x_now;
    double dyl = m_trajectory_points[dist_min_id].y - m_slam_y_now;
    double angle = RoundTheta(m_trajectory_points[dist_min_id].yaw - atan2(dyl, dxl));
    if (angle < 0)
    {
        m_d_now *= -1;
    }

    return m_trajectory_points[dist_min_id];
}

void LqrController::generateSpeed()
{
    // 速度的方向是前进的
    double direction = 1.0;

    for (int i = 0 ; i < m_trajectory_points.size()-1; i++)
    {
        double dyaw = abs(m_trajectory_points[i + 1].yaw - m_trajectory_points[i].yaw);  // 每一个点时的yaw变化
        // bool change = (CV_PI / 4.0 <= dyaw <  CV_PI/ 2.0);  // 如果变化的角度在45度至90度之间，变化太快

        // if (change == true)
        // {
        //     direction = -1;
        // }
        // 如果方向不是前进，每个点的速度方向是后退
        if (direction != 1.0)
        {
            m_trajectory_points[i].v = - m_plan_speed_max;
        }
        else
        {
            m_trajectory_points[i].v = m_plan_speed_max;
        }
        // if (change == true)
        // {
        //     m_trajectory_points[i].v = 0.0;
        // }   
    }

    // 最后要降速, 而且有最小速度限制
    for (int i = 0 ; i < 40 ; i++)
    {
        m_trajectory_points[m_trajectory_points.size()-1-i].v = m_plan_speed_max/(50-i); // 越靠近终点，速度越慢
        if (m_trajectory_points[m_trajectory_points.size()-1-i].v <= m_plan_speed_min)
        {
            m_trajectory_points[m_trajectory_points.size()-1-i].v = m_plan_speed_min;
        }      
    }

    // for (int i = 0 ; i < m_trajectory_points.size() ; i++)
    // {
    //     std::cout<<"i: "<<m_trajectory_points[i].v<<std::endl;
    // }

}

void LqrController::setTrajectory(std::vector<cv::Point2d> points)
{
    int size = points.size();
    double s_tmp = 0.0;
    // 为每个路径点设置属性
    for (int i = 0 ; i < points.size(); i++)
    {
        WayPoint p;
        // x,y属性
        p.x = points[i].x; 
        p.y = points[i].y;
        
        // yaw属性
        if (i < points.size()-1)
        {
            p.yaw = atan2(points[i+1].y-points[i].y, points[i+1].x-points[i].x);
        }
        else
        {
            p.yaw = atan2(points[points.size()-1].y-points[points.size()-2].y, 
                points[points.size()-1].x-points[points.size()-2].x);
        }

        // frenet轨迹的s属性
        p.s = s_tmp;
        if (i < points.size()-1)
        {
            s_tmp = s_tmp + sqrt(pow(points[i+1].x-points[i].x,2)+pow(points[i+1].y-points[i].y,2));
        }
        // frenet终点的s
        if (i == points.size() -1 )
        {
            m_s_target = p.s; // s的目标target就是最后一个点的s
        }
        
        // frenet轨迹的d属性
        p.d = 0;

        // frenet轨迹点的弧度属性
        if (i < points.size() -2 )
        {
            double l = sqrt(pow(points[i+1].x-points[i].x,2)+pow(points[i+1].y-points[i].y,2));
            double dtheta = atan2(points[i+2].y-points[i+1].y, points[i+2].x-points[i+1].x)
                - atan2(points[i+1].y-points[i].y, points[i+1].x-points[i].x);
            dtheta = RoundTheta(dtheta);
            double r = abs(l/dtheta);
            p.kappa = 1 / r;
            if (std::isinf(p.kappa) == true)
            {
                p.kappa = 1;
            }
        }
        else
        {
            p.kappa = 0; 
        }
        if (p.kappa > m_kappa_max)
        {
            m_kappa_max = p.kappa; 
        }
        if (p.kappa < m_kappa_min)
        {
            m_kappa_min = p.kappa; 
        }
        // std::cout<<p.kappa<<std::endl;
        
        // v属性
        p.v = 0;
        
        // 添加每个生成的点
        m_trajectory_points.push_back(p);
    }
}

void LqrController::setTrajectory(std::vector<double> x, std::vector<double> y, std::vector<double> s, std::vector<double> yaw, std::vector<double> kappa)
{
    int size = x.size();
    
    m_trajectory_points.clear();
    std::vector<WayPoint>(m_trajectory_points).swap(m_trajectory_points);
    
    // 为每个路径点设置属性
    for (int i = 0 ; i < x.size(); i++)
    {
        WayPoint p;
        // x,y属性
        p.x = x[i]; 
        p.y = y[i];
        
        p.yaw = yaw[i];
        p.s = s[i];
       
        
        // frenet轨迹的d属性
        p.d = 0;
        p.kappa = kappa[i];
        // v属性
        p.v = 0;
        
        // 添加每个生成的点
        m_trajectory_points.push_back(p);
    }
}

void LqrController::receiveSlamPose(double x, double y, double theta)
{
    m_slam_x_now = x;
    m_slam_y_now = y;
    m_slam_theta_now = theta;
    // std::cout<<"轨迹规划收到"<<m_slam_x_now<<" "<<m_slam_y_now<<" "<<m_slam_theta_now<<std::endl;
}

double LqrController::getControl_v()
{
    return m_ref_point.v;
}

double LqrController::getControl_omega()
{
    return m_control_omega;
}

double LqrController::getControl_s()
{
    return m_ref_point.s;
}

double LqrController::getTrajectory_s_target()
{
    return m_trajectory_points[m_trajectory_points.size()-1].s;
}
#include "DockingPlan.h"

extern double RoundTheta(double angle);

DockingPlan::DockingPlan()
{
    m_matrix_size = 2000;
    m_map_resolution = 0.005; // 地图分辨率

    m_robot_v_max = 0.2; // m/s
    m_robot_v_min = 0.01 ; // m/s
    m_robot_omega_max = 0.5; // radian/s
    m_robot_omega_min = 0.01; // radian/s

    m_start_spline_time = 10; // s
    m_end_spline_time = 10; // s

    m_phase = 0;
}

DockingPlan::~DockingPlan()
{

}

double DockingPlan::getTimeNow()
{
    std::chrono::system_clock::time_point current_time = std::chrono::system_clock::now();
    std::chrono::seconds sec = std::chrono::duration_cast<std::chrono::seconds>(current_time.time_since_epoch());
    std::chrono::nanoseconds nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(current_time.time_since_epoch());

    return nsec.count()*10e-10;
}

void DockingPlan::init()
{
    clearCache();
    m_phase = 0;
}

int DockingPlan::initPath(double x1, double y1, double theta1, double x2, double y2, double theta2)
{
    m_x_initial = x1;
    m_y_initial = y1;
    m_theta_initial = theta1;
    m_x_target = x2;
    m_y_target = y2;
    m_theta_target = theta2;

    if (adjustSpeed(m_x_initial, m_y_initial, m_theta_initial, m_x_target, m_y_target, m_theta_target) == -1)
    {
        return -1;
    }
 
    m_lqrController.setConfig(m_map_resolution, m_robot_v_max, m_robot_v_min, m_robot_omega_max, m_robot_omega_min);

    calculateTracjectoryPoints();
    
    m_lqrController.setTrajectory(m_cb.m_x, m_cb.m_y, m_cb.m_s, m_cb.m_yaw, m_cb.m_kappa);
    m_lqrController.generateSpeed();
    m_s_target = m_lqrController.getTrajectory_s_target();

    m_timestamp_now = getTimeNow();
    m_timestamp_last = getTimeNow();

    return 1;
}

// SLAM地图坐标系下的点转为LQR控制坐标系下的点
void DockingPlan::calculateTracjectoryPoints()
{
    std::vector<cv::Point2d> input = generateWaypoint(0, 0, m_theta_initial,
                                                    m_x_target - m_x_initial, m_y_target - m_y_initial, m_theta_target);
    for (int i = 0 ; i < input.size(); i ++)
    {
        double x_ = m_matrix_size/2 + input[i].x/m_map_resolution;
        double y_ = m_matrix_size/2 - input[i].y/m_map_resolution;
        std::cout<<input[i].x<<"  "<<input[i].y<<std::endl;
        std::cout<<x_<<"  "<<y_<<std::endl;
        m_waypoints.push_back(cv::Point2d(x_, y_));
    }

    m_cb.setTrajectory(m_waypoints);
}

// 生成SLAM坐标系下的路点
std::vector<cv::Point2d> DockingPlan::generateWaypoint(double start_x, double start_y, double start_theta, double end_x, double end_y, double end_theta)
{   
    double x_initial_after = start_x + m_robot_v_max*m_start_spline_time*cos(start_theta);
    double y_initial_after = start_y + m_robot_v_max*m_start_spline_time*sin(start_theta);
    double x_target_before = end_x + m_robot_v_max*m_end_spline_time*cos(CV_PI+end_theta);
    double y_target_before = end_y + m_robot_v_max*m_end_spline_time*sin(CV_PI+end_theta);

    std::vector<cv::Point2d> waypoints;

    if (m_planType == "pass")
    {
        waypoints.push_back(cv::Point2d(start_x, start_y));
        waypoints.push_back(cv::Point2d(x_initial_after, y_initial_after));
        waypoints.push_back(cv::Point2d(x_target_before, y_target_before));
        // for (int i = m_end_spline_time * 9 ; i > 0 ; i--)
        // {
        //     double x_ = end_x + m_robot_v_max*i*0.1*cos(CV_PI+end_theta);
        //     double y_ = end_y + m_robot_v_max*i*0.1*sin(CV_PI+end_theta);
        //     waypoints.push_back(cv::Point2d(x_, y_));
        // }
        // waypoints.push_back(cv::Point2d(end_x, end_y));
    }
    if (m_planType == "line")
    {
        waypoints.push_back(cv::Point2d(start_x, start_y));
        waypoints.push_back(cv::Point2d(x_target_before, y_target_before));
    }
    if (m_planType == "end")
    {
        waypoints.push_back(cv::Point2d(x_target_before, y_target_before));
        waypoints.push_back(cv::Point2d(end_x, end_y));
    }

    return waypoints;
}

// 判断两点之间的路径能否规划，并且根据路径的长度和方向调整速度和角速度
// 输入数据单位（m），和机器人在图像上的角度（弧度）
int DockingPlan::adjustSpeed(double x_initial, double y_initial, double theta_initial, double x_target, double y_target, double theta_target)
{
    double dist = sqrt(pow(x_target - x_initial, 2) + pow(y_target - y_initial, 2));
    double angle = (theta_target - theta_initial > CV_PI )? (2*CV_PI - theta_target + theta_initial): (theta_target - theta_initial);
    if (dist < 0.5 && angle > CV_PI/6)
    {
        return -1;
    }
    
    // m_robot_v_max = ;
    // m_robot_omega_max = ;

    return 0;
}


// 对接控制核心函数
int DockingPlan::dockingPlanning(bool able_forward, bool able_reverse, bool able_left_spin, bool able_right_spin,
                            double x_start, double y_start, double theta_start,
                            double x_end, double y_end, double theta_end,
                            double robot_x_now, double robot_y_now, double robot_theta_now,
                            double& v, double& omega, std::string planType, bool isdraw)
{  
    m_planType = planType;
    m_isdraw = isdraw;
    // phase 0: 设置起点和终点
    if (m_phase == 0)
    {
        // 判断起终点能否规划成功
        int s = initPath(x_start, y_start, theta_start, x_end, y_end, theta_end);
        if (s == -1)
        {
            clearCache();
            return 2;
        }
        if (m_isdraw == true)
        {
            showTrajectory();
        }
        m_phase = 1;
    }
    // phase 2: 轨迹跟踪
    if (m_phase == 1)
    {   
        m_robot_x_now = m_matrix_size/2 + (robot_x_now - m_x_initial)/m_map_resolution;
        m_robot_y_now = m_matrix_size/2 - (robot_y_now - m_y_initial)/m_map_resolution;
        m_robot_theta_now = -robot_theta_now;
     
        m_lqrController.receiveSlamPose(m_robot_x_now, m_robot_y_now, m_robot_theta_now);  
        m_lqrController.lqrTrajectoryPlanning();

        double v_control, omega_control, s_control;
        v_control = m_lqrController.getControl_v();
        omega_control = m_lqrController.getControl_omega();
        s_control = m_lqrController.getControl_s();

        // 自己模拟自己更新来画出机器人的估计位置
        m_timestamp_now = getTimeNow(); 
        m_lqrController.update(m_timestamp_now - m_timestamp_last);
        m_timestamp_last = m_timestamp_now;
        m_lqr_x = m_lqrController.getX();
        m_lqr_y = m_lqrController.getY();
        m_lqr_theta = m_lqrController.getTheta(); 

        v = v_control * m_map_resolution;
        omega = omega_control;

        if (m_isdraw == true)
        {
            showRobotPose();
        }

        if ( (!able_forward && v > 0)
        || (!able_reverse && v < 0)
        || (!able_left_spin && omega > 0)
        || (!able_right_spin && omega < 0) )
        {
            v = 0;
            omega = 0;
            return -1;
        }
        double tolerate_reach_range;
        if (m_planType == "pass")
        {
            tolerate_reach_range = 2;
        }
        if (m_planType == "line")
        {
            tolerate_reach_range = 2;
        }
        if (m_planType == "end")
        {
            tolerate_reach_range = 2;
        }
        std::cout<<"!!!!!!!!!!!!!!!!!!!!! "<<std::abs(m_s_target - s_control)<<std::endl;
        if (std::abs(m_s_target - s_control) < tolerate_reach_range)
        {
            if (m_planType == "end")
            {
                std::string path = "/home/airobot/yuchen/error.txt";
                const char* data_path = path.c_str();
                FILE *fp = NULL;
                fp =fopen(data_path,"a");
                fprintf(fp, "%f\t",(robot_x_now-m_x_target));
                fprintf(fp, "%f\t",(robot_y_now-m_y_target));
                double angleError = robot_theta_now - m_theta_target;
                angleError = RoundTheta(angleError);
                fprintf(fp, "%f\n",angleError);
                fclose(fp);
            }
            
            clearCache();
            m_phase = 0;
            return 0;
        }
        else
        {
            return 1;
        }
    }
}

void DockingPlan::showTrajectory()
{
    // 空白图
    m_test = cv::Mat(1.5*m_matrix_size, 1.5*m_matrix_size, CV_8UC3, cv::Scalar(255,255,255));
    
    for (int i = 0 ; i < m_waypoints.size(); i ++)
    {
        cv::circle(m_test, cv::Point((int)(0.5*m_matrix_size/2 + m_waypoints[i].x), (int)(1.5*m_matrix_size/2 - m_waypoints[i].y)), 10, cv::Scalar(255,0,0), 2);
    }
    for (int i = 0; i < m_cb.m_x.size() - 1; i++)
    {
        cv::circle(m_test, cv::Point((int)(0.5*m_matrix_size/2 + m_cb.m_x[i]), (int)(1.5*m_matrix_size/2 - m_cb.m_y[i])), 1, cv::Scalar(255,0,0), -1);
    }
    
}

void DockingPlan::showRobotPose()
{
    // 画出LQR控制模拟的位置
    // cv::circle(m_test, cv::Point((int)(m_lqr_x), (int)(m_lqr_y)), 7, cv::Scalar(200,0,200), 1);            
    // 画出机器人的实际位置
    cv::circle(m_test, cv::Point((int)(0.5*m_matrix_size/2 + m_robot_x_now), (int)(1.5*m_matrix_size/2 - m_robot_y_now)), 7, cv::Scalar(0,0,255), 1);    
    // 画出机器人的参考位置
    cv::circle(m_test, cv::Point((int)(0.5*m_matrix_size/2 + m_lqrController.m_ref_point.x), int(1.5*m_matrix_size/2 - m_lqrController.m_ref_point.y)), 7, cv::Scalar(0,255,0), 1);
    

    resize(m_test, m_image_mini, cv::Size(m_matrix_size/3, m_matrix_size/3)); // 缩小操作
    cv::imshow("mini of original image", m_image_mini);
    cv::waitKey(10);
}

void DockingPlan::clearCache()
{
    m_waypoints.clear();
    std::vector<cv::Point2d>(m_waypoints).swap(m_waypoints);

    m_cb.clear();

    if (m_isdraw)
    {
        m_test = cv::Mat(1.5*m_matrix_size, 1.5*m_matrix_size, CV_8UC3, cv::Scalar(255,255,255));
        m_image_mini = cv::Mat(m_matrix_size/3, m_matrix_size/3, CV_8UC3, cv::Scalar(255,255,255));
        cv::imshow("mini of original image", m_image_mini);
        cv::waitKey(10);
    }  
}
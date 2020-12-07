#include "LqrPlan.h"

LqrPlan::LqrPlan()
{
    m_plan_phase = 0;
    m_isdraw = true;
}

LqrPlan::~LqrPlan()
{

}

void LqrPlan::reset(LqrParams params)
{
    m_dist_start = params.dist_start;
    m_dist_end = params.dist_end;

    m_pathType = params.pathType;

    clearCache();
    m_plan_phase = 0;
}

void LqrPlan::setDist_start(double dist)
{
    m_dist_start = dist;
}

void LqrPlan::setDist_end(double dist)
{
    m_dist_end = dist;
}


int LqrPlan::lqrPlanning(std::string pathType, MapPoint pose_start, MapPoint pose_end, MapPoint pose_now, double& v, double& omega)
{    
    m_pathType = pathType;

    m_plan_v_max = g_robot_status_now.v_max;
    m_plan_v_min = g_robot_status_now.v_min;
    m_plan_omega_max = g_robot_status_now.omega_max;
    m_plan_omega_min = g_robot_status_now.omega_min;

    m_dist_start = 0;
    m_dist_end = 0; 

    // phase 0: 设置起点和终点
    if (m_plan_phase == 0)
    {
        // 判断起终点能否规划成功
        int s = initPath(pose_start, pose_end);
        if (s == -1)
        {
            clearCache();
            return 2;
        }
        outputTrajectory();
        m_plan_phase = 1;
    }
    // phase 2: 轨迹跟踪
    if (m_plan_phase == 1)
    {   
        m_lqrController.receiveSlamPose(g_robot_status_now.x_map, g_robot_status_now.y_map, g_robot_status_now.theta_map);  
        m_lqrController.lqrTrajectoryPlanning();

        double v_control, omega_control, s_control;
        v_control = m_lqrController.getControl_v();
        omega_control = m_lqrController.getControl_omega();
        s_control = m_lqrController.getControl_s();

        // // 自己模拟自己更新来画出机器人的估计位置
        // m_timestamp_now = getTimeNow(); 
        // m_lqrController.update(m_timestamp_now - m_timestamp_last);
        // m_timestamp_last = m_timestamp_now;
        // m_lqr_x = m_lqrController.getX();
        // m_lqr_y = m_lqrController.getY();
        // m_lqr_theta = m_lqrController.getTheta(); 

        v = v_control * g_mapMaintainer.m_map_resolution;
        omega = omega_control;

        // if ( (!able_forward && v > 0)
        // || (!able_reverse && v < 0)
        // || (!able_left_spin && omega > 0)
        // || (!able_right_spin && omega < 0) )
        // {
        //     v = 0;
        //     omega = 0;
        //     return -1;
        // }
        double tolerate_reach_range;
        if (m_pathType == "whole_path_curve")
        {
            tolerate_reach_range = 2;
        }
        if (m_pathType == "whole_path_line")
        {
            tolerate_reach_range = 2;
        }
       
        std::cout<<"!!!!!!!!!!!!!!!!!!!!! "<<std::abs(m_s_target - s_control)<<std::endl;
        if (std::abs(m_s_target - s_control) < tolerate_reach_range)
        {
            if (m_pathType == "end")
            {
                // std::string path = "/home/airobot/yuchen/error.txt";
                // const char* data_path = path.c_str();
                // FILE *fp = NULL;
                // fp =fopen(data_path,"a");
                // fprintf(fp, "%f\t",(robot_x_now-m_x_target));
                // fprintf(fp, "%f\t",(robot_y_now-m_y_target));
                // double angleError = robot_theta_now - m_theta_target;
                // NormalizeAngle(angleError);
                // fprintf(fp, "%f\n",angleError);
                // fclose(fp);
            }
            
            clearCache();
            m_plan_phase = 0;
            return 0;
        }
        else
        {
            return 1;
        }
    }
}

int LqrPlan::initPath(MapPoint pose_start, MapPoint pose_end)
{
    m_pose_start = pose_start;
    m_pose_end = pose_end;

    std::cout<<m_pose_start.x_map<<std::endl;
    std::cout<<m_pose_start.y_map<<std::endl;
    std::cout<<pose_end.x_map<<std::endl;
    std::cout<<pose_end.y_map<<std::endl;

 
    m_lqrController.setConfig(g_mapMaintainer.m_map_resolution, 
        m_plan_v_max, m_plan_v_min, m_plan_omega_max, m_plan_omega_min);

    std::vector<cv::Point2d> input = generateWaypoint();
    for (int i = 0 ; i < input.size(); i ++)
    {
        m_waypoints.push_back(cv::Point2d(input[i].x, input[i].y));
    }

    m_cb.setTrajectory(m_waypoints);
    
    m_lqrController.setTrajectory(m_cb.m_x, m_cb.m_y, m_cb.m_s, m_cb.m_yaw, m_cb.m_kappa);
    m_lqrController.generateSpeed();
    m_s_target = m_lqrController.getTrajectory_s_target();

    // m_timestamp_now = getTimeNow();
    // m_timestamp_last = getTimeNow();

    return 1;
}

std::vector<cv::Point2d> LqrPlan::generateWaypoint()
{   
    std::vector<cv::Point2d> waypoints;

    std::cout<<m_dist_start<<std::endl;
    std::cout<<m_dist_end<<std::endl;

    if (m_pathType == "whole_path_curve")
    {
        double x_initial_after = m_pose_start.x_map + m_dist_start/g_mapMaintainer.m_map_resolution*cos(m_pose_start.theta_map);
        double y_initial_after = m_pose_start.y_map + m_dist_start/g_mapMaintainer.m_map_resolution*sin(m_pose_start.theta_map);
        double x_target_before = m_pose_end.x_map + m_dist_end/g_mapMaintainer.m_map_resolution*cos(CV_PI+m_pose_end.theta_map);
        double y_target_before = m_pose_end.y_map + m_dist_end/g_mapMaintainer.m_map_resolution*sin(CV_PI+m_pose_end.theta_map);
        waypoints.push_back(cv::Point2d(m_pose_start.x_map, m_pose_start.y_map));
        waypoints.push_back(cv::Point2d(x_initial_after, y_initial_after));
        waypoints.push_back(cv::Point2d(x_target_before, y_target_before));
        waypoints.push_back(cv::Point2d(m_pose_end.x_map, m_pose_end.y_map));
    }
    if (m_pathType == "whole_path_line")
    {
        waypoints.push_back(cv::Point2d(m_pose_start.x_map, m_pose_start.y_map));
        waypoints.push_back(cv::Point2d(m_pose_end.x_map, m_pose_end.y_map));
    }

    return waypoints;
}


void LqrPlan::clearCache()
{
    m_waypoints.clear();
    std::vector<cv::Point2d>(m_waypoints).swap(m_waypoints);

    m_cb.clear();

    g_mapMaintainer.clearTrajectory();

}

void LqrPlan::outputTrajectory()
{
    g_mapMaintainer.b_exist_trajectory = true;
    for (int i = 0; i < m_cb.m_x.size() - 1; i++)
    {
        g_mapMaintainer.m_lqrPlan_trajectory_points.push_back(cv::Point(m_cb.m_x[i], m_cb.m_y[i]));
    }
}
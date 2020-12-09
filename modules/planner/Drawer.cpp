#include "Drawer.h"



Drawer::Drawer()
{

}

Drawer::~Drawer()
{

}

void Drawer::draw()
{
    m_show = g_mapMaintainer.m_map_online_whole_empty.clone();

    drawRobot();

    drawTrajectory();
    
    cv::Mat map_local = m_show(
        cv::Rect(g_mapMaintainer.m_map_online_local_col_min, g_mapMaintainer.m_map_online_local_row_min, 
        g_mapMaintainer.m_localMap_size, g_mapMaintainer.m_localMap_size));

    cv::namedWindow( "planner monitor", cv::WINDOW_AUTOSIZE);
    cv::imshow( "planner monitor", map_local);
    cv::waitKey(5);

    clearCache();
}

void Drawer::drawRobot()
{
    cv::circle(m_show,  cv::Point(g_robot_status_now.x_map, g_robot_status_now.y_map), 5, cv::Scalar(0, 0, 0), 1);


    double diagonal_length = sqrt(pow(g_configer.m_robot_length,2)+pow(g_configer.m_robot_width,2)) / 2 / g_mapMaintainer.m_map_resolution;
    double left_up_theta = g_robot_status_now.theta_map - atan(g_configer.m_robot_width/g_configer.m_robot_length);
    NormalizeAngle(left_up_theta);
    m_robot_corners.push_back(cv::Point(int(g_robot_status_now.x_map + diagonal_length*cos(left_up_theta)), 
                                        int(g_robot_status_now.y_map + diagonal_length*sin(left_up_theta))));

    double right_up_theta = g_robot_status_now.theta_map + atan(g_configer.m_robot_width/g_configer.m_robot_length);
    NormalizeAngle(right_up_theta);
    m_robot_corners.push_back(cv::Point(int(g_robot_status_now.x_map + diagonal_length*cos(right_up_theta)), 
                                        int(g_robot_status_now.y_map + diagonal_length*sin(right_up_theta))));

    double right_down_theta = g_robot_status_now.theta_map + CV_PI - atan(g_configer.m_robot_width/g_configer.m_robot_length);
    NormalizeAngle(right_down_theta);
    m_robot_corners.push_back(cv::Point(int(g_robot_status_now.x_map + diagonal_length*cos(right_down_theta)), 
                                        int(g_robot_status_now.y_map + diagonal_length*sin(right_down_theta))));

    double left_down_theta = g_robot_status_now.theta_map + CV_PI + atan(g_configer.m_robot_width/g_configer.m_robot_length);
    NormalizeAngle(left_down_theta);
    m_robot_corners.push_back(cv::Point(int(g_robot_status_now.x_map + diagonal_length*cos(left_down_theta)), 
                                        int(g_robot_status_now.y_map + diagonal_length*sin(left_down_theta))));

    cv::Point center(int(g_robot_status_now.x_map), int(g_robot_status_now.y_map));
    cv::Point center_(int(g_robot_status_now.x_map + 20*cos(g_robot_status_now.theta_map)), int(g_robot_status_now.y_map + 20*sin(g_robot_status_now.theta_map)));
    cv::Point center_left(int(center_.x + 10*cos(g_robot_status_now.theta_map+CV_PI*0.8)), int(center_.y + 10*sin(g_robot_status_now.theta_map+CV_PI*0.8)));
    cv::Point center_right(int(center_.x + 10*cos(g_robot_status_now.theta_map+CV_PI*1.2)), int(center_.y + 10*sin(g_robot_status_now.theta_map+CV_PI*1.2)));

    cv::polylines(m_show, m_robot_corners, true, cv::Scalar(0), 1);
    cv::line(m_show, center, center_, cv::Scalar(0), 1, 8, 0);
    cv::line(m_show, center_, center_left, cv::Scalar(0), 1, 8, 0);
    cv::line(m_show, center_, center_right, cv::Scalar(0), 1, 8, 0);
}

void Drawer::drawTrajectory()
{
    if (g_mapMaintainer.b_exist_trajectory)
    {
        for (int i = 0; i < g_mapMaintainer.m_lqrPlan_trajectory_points.size() - 1; i++)
        {
            cv::circle(m_show, cv::Point((int)(g_mapMaintainer.m_lqrPlan_trajectory_points[i].x), 
                (int)(g_mapMaintainer.m_lqrPlan_trajectory_points[i].y)), 1, cv::Scalar(0, 0, 0), -1);
        }
    }
    
}

void Drawer::clearCache()
{
    m_robot_corners.clear();
    std::vector<cv::Point>(m_robot_corners).swap(m_robot_corners);
}
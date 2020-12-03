#include "MapMaintainer.h"

MapMaintainer::MapMaintainer()
{

}

MapMaintainer::~MapMaintainer()
{

}

int MapMaintainer::setMap(std::string path)
{
    m_map_online_whole_empty = cv::imread( path, cv::IMREAD_COLOR );

    m_map_online_whole = m_map_online_whole_empty.clone();

    m_wholeMap_size_col = m_map_online_whole.cols;
    m_wholeMap_size_row = m_map_online_whole.rows;

    std::cout<<"Map size: ["<<m_wholeMap_size_col<<","<<m_wholeMap_size_row<<"]"<<std::endl;

    m_map_resolution = g_configer.m_map_resolution;

    return 1;
}

bool MapMaintainer::start()
{
	m_thread = (std::thread(std::bind(&MapMaintainer::run, this)));

    
	return true;
}

void MapMaintainer::run()
{
    while(true)
    {
        emptyMap();

        updateRobot();
        
        updateLocalMap();
    
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 时间间隔（毫秒）

        clearCache();
    }
   
}

void MapMaintainer::emptyMap()
{
    m_map_online_whole = m_map_online_whole_empty.clone();
}

void MapMaintainer::updateRobot()
{   
    double diagonal_length = sqrt(pow(g_configer.m_robot_length,2)+pow(g_configer.m_robot_width,2)) / 2 / m_map_resolution;

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

    // for (int i = 0 ; i < m_robot_corners.size(); i++)
    // {
    //     std::cout<<m_robot_corners[i].x<<" "<<m_robot_corners[i].y<<std::endl;
    // }
    cv::polylines(m_map_online_whole, m_robot_corners, true, cv::Scalar(0), 1);
    
    cv::Point center(g_robot_status_now.x_map, g_robot_status_now.y_map);
    cv::Point center_(g_robot_status_now.x_map + 20*cos(g_robot_status_now.theta_map), g_robot_status_now.y_map + 5*sin(g_robot_status_now.theta_map));
    cv::line(m_map_online_whole, center, center_, cv::Scalar(0), 1, 8, 0);
}

void MapMaintainer::updateLocalMap()
{
    

    m_map_online_local = cv::Mat(m_localMap_size, m_localMap_size, CV_32FC1);

    double col_min = (g_robot_status_now.x_map - m_localMap_size/2 > 0) ? g_robot_status_now.x_map - m_localMap_size/2 : 0;
    
    double row_min = (g_robot_status_now.y_map - m_localMap_size/2 > 0) ? g_robot_status_now.y_map - m_localMap_size/2 : 0;
    double col_max = (g_robot_status_now.x_map + m_localMap_size/2 < m_wholeMap_size_col) ? g_robot_status_now.x_map + m_localMap_size/2 : m_wholeMap_size_col - 1;
    double row_max = (g_robot_status_now.y_map + m_localMap_size/2 < m_wholeMap_size_row) ? g_robot_status_now.y_map + m_localMap_size/2 : m_wholeMap_size_row - 1;


    if (col_min == 0)
    {
        col_max = m_localMap_size;
    }
    if (row_min == 0)
    {
        row_max = m_localMap_size;
    }
    if (col_max == m_wholeMap_size_col - 1)
    {
        col_min = m_wholeMap_size_col - 1 - m_localMap_size;
    }
    if (row_max == m_wholeMap_size_row - 1)
    {
        row_min = m_wholeMap_size_row - 1 - m_localMap_size;
    }
    
    cv::Mat map_local = m_map_online_whole(cv::Rect(col_min, row_min, m_localMap_size, m_localMap_size));

    m_map_online_local = map_local.clone();
}

void MapMaintainer::clearCache()
{
    m_robot_corners.clear();
    std::vector<cv::Point>(m_robot_corners).swap(m_robot_corners);
}
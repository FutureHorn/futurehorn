#include "MapMaintainer.h"

MapMaintainer::MapMaintainer()
{
    b_exist_trajectory = false;
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
        updateLocalMap();
    
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 时间间隔（毫秒）

        clearCache();
    }
   
}

void MapMaintainer::updateLocalMap()
{
    // m_map_online_local = cv::Mat(m_localMap_size, m_localMap_size, CV_32FC1);

    double col_min = (g_robot_status_now.x_map - m_localMap_size/2 > 0) ? g_robot_status_now.x_map - m_localMap_size/2 : 0;
    double row_min = (g_robot_status_now.y_map - m_localMap_size/2 > 0) ? g_robot_status_now.y_map - m_localMap_size/2 : 0;
    double col_max = (g_robot_status_now.x_map + m_localMap_size/2 < m_wholeMap_size_col) ? g_robot_status_now.x_map + m_localMap_size/2 : m_wholeMap_size_col - 1;
    double row_max = (g_robot_status_now.y_map + m_localMap_size/2 < m_wholeMap_size_row) ? g_robot_status_now.y_map + m_localMap_size/2 : m_wholeMap_size_row - 1;


    if (col_max == m_wholeMap_size_col - 1)
    {
        col_min = m_wholeMap_size_col - 1 - m_localMap_size;
    }
    if (row_max == m_wholeMap_size_row - 1)
    {
        row_min = m_wholeMap_size_row - 1 - m_localMap_size;
    }
    
    m_map_online_local_col_min = col_min;
    m_map_online_local_row_min = row_min;
    
}

void MapMaintainer::clearTrajectory()
{
    b_exist_trajectory = false;

    m_lqrPlan_trajectory_points.clear();
    std::vector<cv::Point2d>(m_lqrPlan_trajectory_points).swap(m_lqrPlan_trajectory_points);
}

void MapMaintainer::clearCache()
{
    m_robot_corners.clear();
    m_robot_directArrows.clear();
    std::vector<cv::Point>(m_robot_corners).swap(m_robot_corners);
    std::vector<cv::Point>(m_robot_directArrows).swap(m_robot_directArrows);
}
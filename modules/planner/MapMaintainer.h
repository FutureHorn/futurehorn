#ifndef __MAPMAINTAINER_H__
#define __MAPMAINTAINER_H__


#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include "stdlib.h"
#include <thread>

#include "StructDefine.h"
#include "ConfigReader.h"
#include "Common.h"

extern RobotStatus g_robot_status_now;
extern ConfigReader g_configer;

class MapMaintainer
{
    public:
        MapMaintainer();
        ~MapMaintainer();

    public:
        int setMap(std::string path);
        bool start();
        void run();
        void clearCache();

        void emptyMap();
        void updateLocalMap();
        void updateRobot();

    public:
        std::thread m_thread;

    public:
        cv::Mat m_map_online_whole_empty;
        cv::Mat m_map_online_whole;
        cv::Mat m_map_online_local;
        int m_localMap_size = 700;
        double m_map_resolution;

        int m_wholeMap_size_col;
        int m_wholeMap_size_row;

        std::vector<cv::Point> m_robot_corners;

};



#endif
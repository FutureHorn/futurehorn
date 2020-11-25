#ifndef __MAPMAINTAINER_H__
#define __MAPMAINTAINER_H__


#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <string>
#include "stdlib.h"

class MapMaintainer
{
    public:
        MapMaintainer();
        ~MapMaintainer();

    public:
        int setMap(std::string path);

    public:
        cv::Mat m_map_online_whole;
};



#endif
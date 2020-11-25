#include "MapMaintainer.h"

MapMaintainer::MapMaintainer()
{

}

MapMaintainer::~MapMaintainer()
{

}

int MapMaintainer::setMap(std::string path)
{
    m_map_online_whole = cv::imread( path, cv::IMREAD_COLOR ); // Read the file

    return 1;
}

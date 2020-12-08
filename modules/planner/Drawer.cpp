#include "Drawer.h"



Drawer::Drawer()
{

}

Drawer::~Drawer()
{

}

void Drawer::draw()
{
    cv::namedWindow( "planner monitor", cv::WINDOW_AUTOSIZE);
    cv::imshow( "planner monitor", g_mapMaintainer.m_map_online_local);
    cv::waitKey(5);
}
        
void Drawer::clear()
{

}
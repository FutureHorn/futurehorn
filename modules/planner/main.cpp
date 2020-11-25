#include <iostream>


#include "MapMaintainer.h"
#include "ConfigReader.h"
#include "Drawer.h"

const std::string HOME_DIR = getenv("HOME");

MapMaintainer g_mapMaintainer;
Drawer g_drawer;
ConfigReader g_configer;

int main(int argc, char** argv)
{
    if (g_configer.readRobotInfo())
    {
        std::cout<<"robot init ok"<<std::endl;
    }
    if (g_configer.readMapInfo())
    {
        std::cout<<"map init ok"<<std::endl;
    }


    g_mapMaintainer.setMap(g_configer.m_task_map_path);


    std::cout<<"planner init success!"<<std::endl;
    while(true)
    {
        g_drawer.draw();
    }
    
    


    return 1;
}
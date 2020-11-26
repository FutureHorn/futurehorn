#include <iostream>


#include "MapMaintainer.h"
#include "TaskMaintainer.h"
#include "ConfigReader.h"
#include "Drawer.h"

const std::string HOME_DIR = getenv("HOME");

MapMaintainer g_mapMaintainer;
TaskMaintainer_offline g_taskMaintainer_offline;
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
    if (g_configer.readTask_offline())
    {
        std::cout<<"task init ok"<<std::endl;
    }


    g_mapMaintainer.setMap(g_configer.m_task_map_path);
    
    g_taskMaintainer_offline.setTask(g_configer.m_task_type);


    

    std::cout<<"planner init success!"<<std::endl;
    while(true)
    {
        g_drawer.draw();
    }
    
    


    return 1;
}
#ifndef __CONFIGREADER_H__
#define __CONFIGREADER_H__

#include <iostream>
#include <stdlib.h>

#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/filewritestream.h"
#include "rapidjson/prettywriter.h"
#include "rapidjson/filereadstream.h"

extern const std::string HOME_DIR;

const std::string MAP_INFO_JSON_FILENAME(HOME_DIR + "/CodeBase/futurehorn/config/planner/map_info.json");
const std::string ROBOT_INFO_JSON_FILENAME(HOME_DIR + "/CodeBase/futurehorn/config/planner/robot_info.json");
const std::string TASK_OFFLINE_JSON_FILENAME(HOME_DIR + "/CodeBase/futurehorn/config/planner/task_offline.json");

class ConfigReader
{
    public:
        ConfigReader();
        ~ConfigReader();
    
    public:
        int readMapInfo();
        int readRobotInfo();
        int readTask_offline();

    public:
        std::string m_task_map_path;
        double m_map_resolution;

        std::string m_task_type;
        int m_task_start_x;
        int m_task_start_y;
        int m_task_end_x;
        int m_task_end_y;

        double m_robot_length;
        double m_robot_width;

};

#endif
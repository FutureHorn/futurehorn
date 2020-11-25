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

class ConfigReader
{
    public:
        ConfigReader();
        ~ConfigReader();
    
    public:
        int readMapInfo();
        int readRobotInfo();

    public:
        std::string m_task_map_path;

};

#endif
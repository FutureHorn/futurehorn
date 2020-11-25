#include "ConfigReader.h"

ConfigReader::ConfigReader()
{

}

ConfigReader::~ConfigReader()
{

}

int ConfigReader::readMapInfo()
{
    std::cerr << "Read Parameters" << std::endl;

    FILE* fp = fopen(MAP_INFO_JSON_FILENAME.c_str(), "r"); 
    if (fp == NULL)
    {
        std::cerr << "File does not exists!" << std::endl;
        return 0;
    }
    char readBuffer[65536];
    rapidjson::FileReadStream is(fp, readBuffer, sizeof(readBuffer));
    rapidjson::Document doc;
    doc.ParseStream(is);
    fclose(fp);

    int A = doc["map"]["id"].GetInt();
  
	
    std::cout<<"map id : "<<A<<std::endl;

    if(A == 0)
    {
        m_task_map_path = HOME_DIR + "/CodeBase/futurehorn/config/planner/map.png";
    }
    else
    {
        m_task_map_path = HOME_DIR + "/CodeBase/futurehorn/config/planner/map.png";
    }

}
        
int ConfigReader::readRobotInfo()
{

}
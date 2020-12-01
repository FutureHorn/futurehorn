#include "ConfigReader.h"

ConfigReader::ConfigReader()
{

}

ConfigReader::~ConfigReader()
{

}

int ConfigReader::readMapInfo()
{
    std::cerr << "Read map info" << std::endl;

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

    int m_map_id = doc["map"]["id"].GetInt();
  

    if(m_map_id == 0)
    {
        m_task_map_path = HOME_DIR + "/CodeBase/futurehorn/config/planner/map.png";
    }
    else
    {
        m_task_map_path = HOME_DIR + "/CodeBase/futurehorn/config/planner/map.png";
    }

    m_map_resolution = doc["map"]["resolution"].GetDouble();
}
        
int ConfigReader::readRobotInfo()
{
    std::cerr << "Read robot info" << std::endl;

    FILE* fp = fopen(ROBOT_INFO_JSON_FILENAME.c_str(), "r"); 
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

    m_robot_length = doc["robot"]["length"].GetDouble();
    m_robot_width = doc["robot"]["width"].GetDouble();
   
    m_robot_length = m_robot_length * 0.001;
    m_robot_width = m_robot_width * 0.001;
}

int ConfigReader::readTask_offline()
{
    std::cerr << "Read offine task" << std::endl;

    FILE* fp = fopen(TASK_OFFLINE_JSON_FILENAME.c_str(), "r"); 
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

    std::string m_task_type = doc["task"]["type"].GetString();
    int m_task_start_x = doc["task"]["start"]["x"].GetInt();
    int m_task_start_y = doc["task"]["start"]["y"].GetInt();
    int m_task_end_x = doc["task"]["end"]["x"].GetInt();
    int m_task_end_y = doc["task"]["end"]["y"].GetInt();


}
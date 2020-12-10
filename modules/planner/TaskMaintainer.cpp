#include "TaskMaintainer.h"

TaskMaintainer::TaskMaintainer()
{
    
}

TaskMaintainer::~TaskMaintainer()
{

}

TaskMaintainer_offline::TaskMaintainer_offline()
{
    m_tasks_current_id = -1;
}

TaskMaintainer_offline::~TaskMaintainer_offline()
{

}

int TaskMaintainer_offline::loadFile()
{
    std::cerr << "Read offine task" << std::endl;

    FILE* fp = fopen((HOME_DIR+TASK_OFFLINE_JSON_FILENAME).c_str(), "r"); 
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


    m_tasks_num= doc["tasks"].Size();
    for (int i = 0; i < m_tasks_num; i++) 
    {
        rapidjson::StringBuffer buffer;
        rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
        doc["tasks"][i].Accept(writer);
        std::cout << buffer.GetString() << std::endl;
        m_tasks_buffer.push_back(buffer.GetString());

    }  
    return 0;
}

Task TaskMaintainer_offline::getNextTask()
{
    m_tasks_current_id += 1;
    std::string task_name = extract_value_string({"type"}, m_tasks_buffer[m_tasks_current_id]);

    if (task_name == "MOVE")
    {
        m_subtasks_num = 2;

        m_task_current.type = TASK_MOVE;
        m_task_current.x_target = extract_value_double({"target_pose","x"}, m_tasks_buffer[m_tasks_current_id]);
        m_task_current.y_target = extract_value_double({"target_pose","y"}, m_tasks_buffer[m_tasks_current_id]);
        m_task_current.theta_target = extract_value_double({"target_pose","theta"}, m_tasks_buffer[m_tasks_current_id]); 
    }
    return m_task_current;
}  

std::string TaskMaintainer_offline::extract_value_string(std::vector<std::string> key, std::string buffer)
{
    char* json = (char*)buffer.c_str();
	rapidjson::Document document;
	document.Parse(json);
	
    std::string s;
    
    if (key.size() == 1)
    {
        s = document[key[0].c_str()].GetString();
    }
    if (key.size() == 2)
    {
        s = document[key[0].c_str()][key[1].c_str()].GetString();
    }
    if (key.size() == 3)
    {
        s = document[key[0].c_str()][key[1].c_str()][key[2].c_str()].GetString();
    }
	
    // std::cout<<"extract: "<<s<<std::endl;	

    return s;
}

double TaskMaintainer_offline::extract_value_double(std::vector<std::string> key, std::string buffer)
{
    char* json = (char*)buffer.c_str();
	rapidjson::Document document;
	document.Parse(json);
	
    double d;
	if (key.size() == 1)
    {
        d = document[key[0].c_str()].GetDouble();
    }
    if (key.size() == 2)
    {
        d = document[key[0].c_str()][key[1].c_str()].GetDouble();
    }
    if (key.size() == 3)
    {
        d = document[key[0].c_str()][key[1].c_str()][key[2].c_str()].GetDouble();
    }
    // std::cout<<"extract: "<<d<<std::endl;	

    return d;
}
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
}


Task TaskMaintainer_offline::getNextTask()
{
    m_tasks_current_id += 1;
    std::string task_name = extract_value("type", m_tasks_buffer[m_tasks_current_id]);
    if (task_name == "MOVE")
    {
        m_subtasks_num = 2;

        m_task_current.type = TASK_MOVE;
        m_task_current.x_target = String_to_Double(extract_value("x", m_tasks_buffer[m_tasks_current_id]));
        m_task_current.y_target = String_to_Double(extract_value("y", m_tasks_buffer[m_tasks_current_id]));
        m_task_current.theta_target = String_to_Double(extract_value("theta", m_tasks_buffer[m_tasks_current_id])); 
    }
    return m_task_current;
}  

std::string TaskMaintainer_offline::extract_value(std::string key, std::string buffer)
{
  
	int key_start = buffer.find(key);

	int value_start = key_start + key.length() + 1;
	int value_end = value_start;
	while (true)
	{
		if (buffer[value_end+1] == ','|| buffer[value_end+1] == '}')
		{
			break;
		}
		value_end += 1;
	}
    
    std::string result = buffer.substr(value_start, value_end - value_start + 1);
	std::cout<<result<<std::endl;	

    return result;
}
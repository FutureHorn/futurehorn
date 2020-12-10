#ifndef __TASKMAINTAINER_H__
#define __TASKMAINTAINER_H__

#include <iostream>
#include <vector>

#include "StructDefine.h"
#include "ConfigReader.h"

extern const std::string HOME_DIR;
const std::string TASK_OFFLINE_JSON_FILENAME("/CodeBase/futurehorn/config/planner/task_offline.json");

class TaskMaintainer
{
    public:
        TaskMaintainer();
        ~TaskMaintainer();
};

class TaskMaintainer_offline
{
    public:
        TaskMaintainer_offline();
        ~TaskMaintainer_offline();

    public:
        int loadFile();
        Task getNextTask();
        std::string extract_value_string(std::vector<std::string> key, std::string buffer);
        double extract_value_double(std::vector<std::string> key, std::string buffer);

    public:
        std::vector<std::string> m_tasks_buffer;
        int m_tasks_num;
        int m_subtasks_num;

        int m_tasks_current_id;

        TASKTYPE m_tasktype;
        Task m_task_current;

};


#endif
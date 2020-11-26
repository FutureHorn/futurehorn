#ifndef __TASKMAINTAINER_H__
#define __TASKMAINTAINER_H__

#include <iostream>

#include "StructDefine.h"

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
        void setTask(std::string taskName);

    public:
        TASKTYPE m_tasktype;
};


#endif
#include "TaskMaintainer.h"

TaskMaintainer::TaskMaintainer()
{

}

TaskMaintainer::~TaskMaintainer()
{

}

TaskMaintainer_offline::TaskMaintainer_offline()
{

}

TaskMaintainer_offline::~TaskMaintainer_offline()
{

}

void TaskMaintainer_offline::setTask(std::string taskName)
{
    if (taskName == "MOVE")
    {
        m_tasktype == TASK_MOVE;
    }
}
#ifndef __DRAWER_H__
#define __DRAWER_H__

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <thread>

#include "MapMaintainer.h"
#include "StructDefine.h"

extern MapMaintainer g_mapMaintainer;
extern RobotStatus g_robotStatus_now;

class Drawer
{
    public:
        Drawer();
        ~Drawer();


    public:
        void draw();
        void clear();

    public:
        std::thread m_thread;
};

#endif
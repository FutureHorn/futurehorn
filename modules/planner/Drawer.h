#ifndef __DRAWER_H__
#define __DRAWER_H__

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include "MapMaintainer.h"

extern MapMaintainer g_mapMaintainer;

class Drawer
{
    public:
        Drawer();
        ~Drawer();

    public:
        void draw();
        void clear();
};

#endif
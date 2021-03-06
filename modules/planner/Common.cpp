#include "Common.h"

void NormalizeAngle(double &angle)
{
    double a = fmod(angle + CV_PI, 2.0 * CV_PI);
    if (a < 0.0)
    {
        a += (2.0 * CV_PI);
    }
    angle = a - CV_PI;
}

double String_to_Double(std::string s)
{
    double p = atof(s.c_str());
    return p;
} 
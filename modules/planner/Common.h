#ifndef __COMMON_H__
#define __COMMON_H__

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include<string>

void NormalizeAngle(double &angle);

double String_to_Double(std::string s);

#endif
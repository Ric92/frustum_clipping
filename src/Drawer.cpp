#ifndef DRAWER_CPP_
#define DRAWER_CPP_
#include "Drawer.h"
template <>
Drawer<pcl::PointXYZRGB> *Drawer<pcl::PointXYZRGB>::mSingleton = nullptr;
#endif

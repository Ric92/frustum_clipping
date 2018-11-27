#include <iostream>
#include <fstream>
#include <string>
#include <Eigen/Eigen>
#include <iomanip>
#include <sstream>
#include <vector>
#include <iterator>
#include <algorithm>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/plot.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/filters/frustum_culling.h>

#include "include/Clipper.h"
#include "include/Drawer.h"
#include "src/Frustum.cpp"
#include "src/Drawer.cpp"

int main(int argc, char **argv)
{
    Drawer<pcl::PointXYZRGB>::init();
    auto draw = Drawer<pcl::PointXYZRGB>::get();
    Clipper a;
    Eigen::Matrix4f b;
    b.setIdentity();
    Frustum c(b, 45, 45, 1, 5);
    bool end = true;
    while (end)
    {
        draw->spinOnce();
    }
    std::cout << "Closing \n";
    return (0);
}
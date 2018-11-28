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

#include "Clipper.h"
#include "Drawer.h"
#include "Frustum.h"

int main(int argc, char **argv)
{
    Drawer<pcl::PointXYZRGB>::init();
    auto draw = Drawer<pcl::PointXYZRGB>::get();
    Clipper a;
    Eigen::Matrix4f b;
    b.setIdentity();
    std::shared_ptr<Frustum> f1(new Frustum(b, 45, 45, 1, 5));
    draw->frustum(1,f1);
    std::vector<Eigen::Vector3f> planePoints;
    planePoints.push_back(f1->mFpTopLeft);
    planePoints.push_back(f1->mFpTopRight);
    planePoints.push_back(f1->mFpBotLeft);
    planePoints.push_back(f1->mFpBotRight);

    std::vector<Eigen::Vector3f> planePoints2;
    planePoints2.push_back(f1->mNpTopLeft);
    planePoints2.push_back(f1->mNpTopRight);
    planePoints2.push_back(f1->mNpBotLeft);
    planePoints2.push_back(f1->mNpBotRight);
    
    draw->plane(1,f1->mFplane,planePoints);
    draw->plane(2,f1->mNplane,planePoints2);
    bool end = true;
    while (end)
    {
        draw->spinOnce();
    }
    std::cout << "Closing \n";
    return (0);
}
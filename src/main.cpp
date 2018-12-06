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
#include <pcl/surface/convex_hull.h>

#include "Drawer.h"
#include "Frustum.h"

int main(int argc, char **argv)
{
    Drawer<pcl::PointXYZRGB>::init();
    auto draw = Drawer<pcl::PointXYZRGB>::get();

    Eigen::Matrix4f pose1;
    pose1 << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    std::shared_ptr<Frustum> f1(new Frustum(1, pose1, 45, 45, 1, 5));

    Eigen::Matrix4f pose2;
    pose2 << 1, 0, 0, 1,
        0, 0.5, 0, 1,
        0, 0, 1, 2,
        0, 0, 0, 1;
    std::shared_ptr<Frustum> f2(new Frustum(2, pose2, 45, 45, 1, 5));

    draw->polyhedron(f1->id, f1->mPosition, f1->getFacets());
    draw->polyhedron(f2->id, f2->mPosition, f2->getFacets());

    std::vector<Eigen::Vector3f> inter;
    f1->clipConvexPolyhedron(f2, inter);
    std::cout << "Number of intersection points: " << inter.size() << "\n";

    std::string v = "Intersection_point_";
    int i = 0;
    for (auto point : inter)
    {
        draw->point(point, v + std::to_string(i));
        i++;
    }

    std::cout << "Convex hull has " << f1->computeVolumeFromPoints(inter) << " m³ \n";
    std::cout << "Volume of frustum: " << f1->getVolume() << " m³ \n";
    std::cout << "% of volume intersected " << 100*(f1->computeVolumeFromPoints(inter)/f1->getVolume()) << "  \n";

    std::cout << "Clipping ended \n";
    bool end = true;
    while (end)
    {
        draw->spinOnce();
    }
    std::cout << "Closing \n";
    return (0);
}
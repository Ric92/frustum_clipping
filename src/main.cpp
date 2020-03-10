#include <Eigen/Eigen>
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

#include <pcl/console/parse.h>
#include <pcl/filters/frustum_culling.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "Cube.h"
#include "Drawer.h"
#include "Frustum.h"

int main(int argc, char **argv) {
    auto start = std::chrono::high_resolution_clock::now();
    Drawer<pcl::PointXYZRGB>::init();
    auto draw = Drawer<pcl::PointXYZRGB>::get();

    Eigen::Matrix4f pose1;
    pose1 << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
    std::shared_ptr<Cube> c1(new Cube(1, pose1, 10, 10, 10));

    Eigen::Matrix4f pose2;
    pose2 << 0.7071068, 0, 0.7071068, 1, 0, 1, 0, 1, -0.7071068, 0, 0.7071068,
        2, 0, 0, 0, 1;
    std::shared_ptr<Cube> c2(new Cube(2, pose2, 15, 5, 10));

    draw->polyhedron(c1->id, c1->mPose.block(0, 3, 3, 1), c1->getFacets());
    draw->polyhedron(c2->id, c2->mPose.block(0, 3, 3, 1), c2->getFacets());

    std::vector<Eigen::Vector3f> inter;
    std::vector<Eigen::Vector3f> culledInter;
    c2->clipConvexPolyhedron(c1, inter);

    std::cout << "Number of intersection points: " << inter.size() << "\n";

    // for (auto &p : inter) {
    //     bool duplicated = false;
    //     for (auto &c : culledInter) {
    //         if (p == c) {
    //             duplicated = true;
    //             break;
    //         }
    //     }
    //     if (!duplicated) culledInter.push_back(p);
    // }
    // std::cout << "Number of intersection points: " << culledInter.size()
    //           << "\n";

    std::string v = "Intersection_point_";
    int i = 0;
    for (auto point : inter) {
        std::cout << " Point " << i << " : " << point(0) << " " << point(1)
                  << " " << point(2) << std::endl;
        draw->point(point, v + std::to_string(i));
        i++;
    }

    std::cout << "Convex hull has " << c1->computeVolumeFromPoints(inter)
              << " m³ \n";
    std::cout << "Volume of frustum: " << c1->getVolume() << " m³ \n";
    std::cout << "% of volume intersected "
              << 100 * (c1->computeVolumeFromPoints(inter) / c1->getVolume())
              << "  \n";
    
    std::cout << "Volume of frustum: " << c1->computeVolumeFromPoints(c1->getVertices()) << " m³ \n";
        std::cout << "% of volume intersected "
              << 100 * (c1->computeVolumeFromPoints(inter) / c1->computeVolumeFromPoints(c1->getVertices()))
              << "  \n";
    
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<float, std::milli> duration = (end - start);
    std::cout << "Time elapsed: " << duration.count() / 1000 << std::endl;

    // std::cout << "Clipping ended \n";
    bool run = true;
    while (run) {
        draw->spinOnce();
    }
    std::cout << "Closing \n";
    return (0);
}

// int main(int argc, char **argv)
// {
//     Drawer<pcl::PointXYZRGB>::init();
//     auto draw = Drawer<pcl::PointXYZRGB>::get();

//     Eigen::Matrix4f pose1;
//     pose1 << 1, 0, 0, 0,
//         0, 1, 0, 0,
//         0, 0, 1, 0,
//         0, 0, 0, 1;
//     std::shared_ptr<Frustum> f1(new Frustum(1, pose1, 45, 45, 1, 5));

//     Eigen::Matrix4f pose2;
//     pose2 << 1, 0, 0, 1,
//         0, 0.5, 0, 1,
//         0, 0, 1, 2,
//         0, 0, 0, 1;
//     std::shared_ptr<Frustum> f2(new Frustum(2, pose2, 45, 45, 1, 5));

//     draw->polyhedron(f1->id, f1->mPosition, f1->getFacets());
//     draw->polyhedron(f2->id, f2->mPosition, f2->getFacets());

//     std::vector<Eigen::Vector3f> inter;
//     f1->clipConvexPolyhedron(f2, inter);
//     std::cout << "Number of intersection points: " << inter.size() << "\n";

//     std::string v = "Intersection_point_";
//     int i = 0;
//     for (auto point : inter)
//     {
//         draw->point(point, v + std::to_string(i));
//         i++;
//     }

//     std::cout << "Convex hull has " << f1->computeVolumeFromPoints(inter) <<
//     " m³ \n"; std::cout << "Volume of frustum: " << f1->getVolume() << " m³
//     \n"; std::cout << "% of volume intersected " <<
//     100*(f1->computeVolumeFromPoints(inter)/f1->getVolume()) << "  \n";

//     std::cout << "Clipping ended \n";
//     bool end = true;
//     while (end)
//     {
//         draw->spinOnce();
//     }
//     std::cout << "Closing \n";
//     return (0);
// }
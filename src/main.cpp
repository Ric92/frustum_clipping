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
    Clipper clip;
    Eigen::Matrix4f pose1;
    pose1 << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1, 0,
             0, 0, 0, 1;
    std::shared_ptr<Frustum> f1(new Frustum(1, pose1, 45, 45, 1, 5));
    draw->frustum(f1);

    Eigen::Matrix4f pose2;
    pose2 << 1, 0, 0, 2,
             0, 1, 0, 2,
             0, 0, 1, 2,
             0, 0, 0, 1;
    std::shared_ptr<Frustum> f2(new Frustum(2, pose2, 45, 45, 1, 5));
    draw->frustum(f2);
    std::vector<Eigen::Vector3f> inter;
    clip.clipFrustumFrustum(f1,f2,inter);
    clip.clipFrustumFrustum(f2,f1,inter);
    
    //First test
    Eigen::Vector3f v1(2, 0, 2);     //Right-Left line
    Eigen::Vector3f v2(4, 1, -2);

    draw->line(std::make_pair(v2, v1), "randomline");
    clip.clipSegmentFrustum(f1, std::make_pair(v2, v1),"randomline");

    Eigen::Vector3f v3(2, 0, 2);        // Right-up line
    Eigen::Vector3f v4(4, 4, -2);

    draw->line(std::make_pair(v3, v4), "randomline2");
    clip.clipSegmentFrustum(f1, std::make_pair(v3, v4),"randomline2");

    Eigen::Vector3f v5(4, 1, -2);        // Right-up line
    Eigen::Vector3f v6(4, 4, -2);

    draw->line(std::make_pair(v5, v6), "randomline3");
    clip.clipSegmentFrustum(f1, std::make_pair(v5, v6),"randomline3");

    // Eigen::Vector3f v1(0, 0, 0);     //Right-Left line
    // Eigen::Vector3f v2(8, 0, 0);

    // draw->line(std::make_pair(v2, v1), "randomline");
    // clip.clipSegmentFrustum(f1, std::make_pair(v2, v1),"randomline");

    // Eigen::Vector3f v3(3, 0, 1);        // Right-up line
    // Eigen::Vector3f v4(0, 2, 3);

    // draw->line(std::make_pair(v3, v4), "randomline2");
    // clip.clipSegmentFrustum(f1, std::make_pair(v3, v4),"randomline2");

    // Eigen::Vector3f v5(2, 1,0);        // Right-up line
    // Eigen::Vector3f v6(3, 1, 0);

    // draw->line(std::make_pair(v5, v6), "randomline3");
    // clip.clipSegmentFrustum(f1, std::make_pair(v5, v6),"randomline3");

    bool end = true;
    while (end)
    {
        draw->spinOnce();
    }
    std::cout << "Closing \n";
    return (0);
}
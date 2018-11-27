#include <iostream>
#include <opencv2/core/eigen.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

template <typename PointType_>
inline Drawer<PointType_>::Drawer()
{
    mViewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer("3D Viewer"));
    mViewer->setBackgroundColor(100, 100, 100, 0);
    mViewer->addCoordinateSystem(0.05, "base", 0);
    mViewer->addCoordinateSystem(0.02, "current_pose", 0);
    mViewer->registerKeyboardCallback(&Drawer::keycallback, *this, (void *)&mViewer);
    mViewer->setCameraPosition(1.59696, 0.285761, -3.40482, -0.084178, -0.989503, -0.117468);
}

//---------------------------------------------------------------------------------------------------------------------
template <typename PointType_>
inline void Drawer<PointType_>::keycallback(const pcl::visualization::KeyboardEvent &_event, void *_data)
{
    if (_event.keyDown() && _event.getKeySym() == "z")
    {
        std::cout << "[Visualizer] Toogle pause" << std::endl;
        if (mPause == true)
            mPause = false;
        else
            pause();
    }
};
//---------------------------------------------------------------------------------------------------------------------
template <typename PointType_>
inline void Drawer<PointType_>::spinOnce()
{
    if (!mViewer)
        return;

    mViewer->spinOnce(10, true);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    cv::waitKey(10);
}
//---------------------------------------------------------------------------------------------------------------------
template <typename PointType_>
inline bool Drawer<PointType_>::init()
{
    mSingleton = new Drawer();
    return true;
}
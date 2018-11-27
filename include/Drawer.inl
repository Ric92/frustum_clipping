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
inline bool Drawer<PointType_>::init()
{
    mSingleton = new Drawer();
    return true;
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
}
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
inline void Drawer<PointType_>::line()
{
}
//---------------------------------------------------------------------------------------------------------------------
template <typename PointType_>
inline void Drawer<PointType_>::frustum(int id, std::shared_ptr<Frustum>_frustum)
{
    auto position = eigenVector3fToPcl(_frustum->mPosition);
    auto ftl = eigenVector3fToPcl(_frustum->mFpTopLeft);
    auto ftr = eigenVector3fToPcl(_frustum->mFpTopRight);
    auto fbl = eigenVector3fToPcl(_frustum->mFpBotLeft);
    auto fbr = eigenVector3fToPcl(_frustum->mFpBotRight);

    auto ntl = eigenVector3fToPcl(_frustum->mNpTopLeft);
    auto ntr = eigenVector3fToPcl(_frustum->mNpTopRight);
    auto nbl = eigenVector3fToPcl(_frustum->mNpBotLeft);
    auto nbr = eigenVector3fToPcl(_frustum->mNpBotRight);

    mViewer->addCoordinateSystem(1.0, _frustum->mPosition[0], _frustum->mPosition[1], _frustum->mPosition[2], "frustum" + std::to_string(id));

    mViewer->addLine(position, ftl, 0.0, 0.0, 1.0, "frustum_ftl" + std::to_string(id));
    mViewer->addLine(position, ftr, 0.0, 0.0, 1.0, "frustum_ftr" + std::to_string(id));
    mViewer->addLine(position, fbl, 1.0, 0.0, 0.0, "frustum_nearbotleft" + std::to_string(id));
    mViewer->addLine(position, fbr, 1.0, 0.0, 0.0, "frustum_nbr" + std::to_string(id));

    mViewer->addLine(ftl, ftr, 1.0, 0.0, 0.0, "ftl_ftr" + std::to_string(id));
    mViewer->addLine(ftl, fbl, 1.0, 0.0, 0.0, "ftl_fbl" + std::to_string(id));
    mViewer->addLine(fbr, ftr, 1.0, 0.0, 0.0, "fbr_ftr" + std::to_string(id));
    mViewer->addLine(fbr, fbl, 1.0, 0.0, 0.0, "fbr_fbl" + std::to_string(id));

    mViewer->addLine(ntl, ntr, 1.0, 0.0, 0.0, "ntl_ntr" + std::to_string(id));
    mViewer->addLine(ntl, nbl, 1.0, 0.0, 0.0, "ntl_nbl" + std::to_string(id));
    mViewer->addLine(nbr, ntr, 1.0, 0.0, 0.0, "nbr_ntr" + std::to_string(id));
    mViewer->addLine(nbr, nbl, 1.0, 0.0, 0.0, "nbr_nbl" + std::to_string(id));
}
//---------------------------------------------------------------------------------------------------------------------
template <typename PointType_>
inline PointType_ Drawer<PointType_>::eigenVector3fToPcl(Eigen::Vector3f _vec)
{
    PointType_ output;
    output.x = _vec[0];
    output.y = _vec[1];
    output.z = _vec[2];
    return output;
}
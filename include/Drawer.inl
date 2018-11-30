#include <iostream>
#include <Eigen/Eigen>
#include <pcl/io/pcd_io.h>
#include <pcl/geometry/planar_polygon.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/PolygonMesh.h>
#include <pcl/surface/convex_hull.h>

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
inline void Drawer<PointType_>::frustum(int id, std::shared_ptr<Frustum> _frustum)
{
    mViewer->addCoordinateSystem(1.0, _frustum->mPosition[0], _frustum->mPosition[1], _frustum->mPosition[2], "frustum" + std::to_string(id));

    // auto position = eigenVector3fToPcl(_frustum->mPosition);
    // auto ftl = eigenVector3fToPcl(_frustum->mFpTopLeft);
    // auto ftr = eigenVector3fToPcl(_frustum->mFpTopRight);
    // auto fbl = eigenVector3fToPcl(_frustum->mFpBotLeft);
    // auto fbr = eigenVector3fToPcl(_frustum->mFpBotRight);

    // auto ntl = eigenVector3fToPcl(_frustum->mNpTopLeft);
    // auto ntr = eigenVector3fToPcl(_frustum->mNpTopRight);
    // auto nbl = eigenVector3fToPcl(_frustum->mNpBotLeft);
    // auto nbr = eigenVector3fToPcl(_frustum->mNpBotRight);

    // mViewer->addLine(position, ftl, 0.0, 0.0, 1.0, "frustum_ftl" + std::to_string(id));
    // mViewer->addLine(position, ftr, 0.0, 0.0, 1.0, "frustum_ftr" + std::to_string(id));
    // mViewer->addLine(position, fbl, 1.0, 0.0, 0.0, "frustum_nearbotleft" + std::to_string(id));
    // mViewer->addLine(position, fbr, 1.0, 0.0, 0.0, "frustum_nbr" + std::to_string(id));

    // mViewer->addLine(ftl, ftr, 1.0, 0.0, 0.0, "ftl_ftr" + std::to_string(id));
    // mViewer->addLine(ftl, fbl, 1.0, 0.0, 0.0, "ftl_fbl" + std::to_string(id));
    // mViewer->addLine(fbr, ftr, 1.0, 0.0, 0.0, "fbr_ftr" + std::to_string(id));
    // mViewer->addLine(fbr, fbl, 1.0, 0.0, 0.0, "fbr_fbl" + std::to_string(id));

    // mViewer->addLine(ntl, ntr, 1.0, 0.0, 0.0, "ntl_ntr" + std::to_string(id));
    // mViewer->addLine(ntl, nbl, 1.0, 0.0, 0.0, "ntl_nbl" + std::to_string(id));
    // mViewer->addLine(nbr, ntr, 1.0, 0.0, 0.0, "nbr_ntr" + std::to_string(id));
    // mViewer->addLine(nbr, nbl, 1.0, 0.0, 0.0, "nbr_nbl" + std::to_string(id));

    std::vector<Eigen::Vector3f> farPlane;
    farPlane.push_back(_frustum->mFpTopLeft);
    farPlane.push_back(_frustum->mFpTopRight);
    farPlane.push_back(_frustum->mFpBotLeft);
    farPlane.push_back(_frustum->mFpBotRight);

    std::vector<Eigen::Vector3f> nearPlane;
    nearPlane.push_back(_frustum->mNpTopLeft);
    nearPlane.push_back(_frustum->mNpTopRight);
    nearPlane.push_back(_frustum->mNpBotLeft);
    nearPlane.push_back(_frustum->mNpBotRight);

    plane(1, _frustum->mFplane, farPlane);

    plane(2, _frustum->mNplane, nearPlane);
    // addPolygonMesh (const typename pcl::PointCloud< PointT >::ConstPtr &cloud, const std::vector< pcl::Vertices > &vertices, const std::string &id="polygon", int viewport=0)

    mViewer->spinOnce();
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
//---------------------------------------------------------------------------------------------------------------------
template <typename PointType_>
inline void Drawer<PointType_>::plane(int id, Eigen::Vector4f _plane, std::vector<Eigen::Vector3f> _points)
{

    pcl::ModelCoefficients coefficients;
    coefficients.values.resize(4); // We need 4 values
    coefficients.values[0] = _plane[0];
    coefficients.values[1] = _plane[1];
    coefficients.values[2] = _plane[2];
    coefficients.values[3] = _plane[3];
    pcl::PointCloud<pcl::PointXYZRGB> planePoints;
    for (auto point : _points)
    {
        planePoints.push_back(eigenVector3fToPcl(point));
    }
    pcl::PlanarPolygon<pcl::PointXYZRGB> poly; //(planePoints,coefficients);
    poly.setContour(planePoints);
    mViewer->addPolygon(poly, 255, 0, 0, "Poligon" + std::to_string(id), 0);
    mViewer->spinOnce();
}

//---------------------------------------------------------------------------------------------------------------------
template <typename PointType_>
inline void Drawer<PointType_>::meshPlane(int id, Eigen::Vector4f _plane, std::vector<Eigen::Vector3f> _points)
{

    // bool pcl::visualization::PCLVisualizer::addPolygonMesh 	( 	const typename pcl::PointCloud< PointT >::ConstPtr &  	cloud,
    // 	const std::vector< pcl::Vertices > &  	vertices,
    // 	const std::string &  	id = "polygon",
    // 	int  	viewport = 0
    // )
    std::vector<pcl::Vertices> planeVertex;
    pcl::Vertices v;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr planePoints= typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());;
    unsigned int i = 0;
    for (auto point : _points)
    {
        planePoints->push_back(eigenVector3fToPcl(point));
        v.vertices.push_back(i);
        i++;
    }
    planeVertex.push_back(v);

    pcl::ConvexHull<pcl::PointXYZRGB> cHull;
    cHull.setInputCloud(planePoints);

    cHull.reconstruct(*hull,)
    // mViewer->addPolygonMesh(planePoints,planeVertex,"PolMesh"+std::to_string(id),0);

    mViewer->spinOnce();
}

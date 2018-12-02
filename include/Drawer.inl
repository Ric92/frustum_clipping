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
    mViewer->addCoordinateSystem(0.1, "Origin", 0);
    mViewer->registerKeyboardCallback(&Drawer::keycallback, *this, (void *)&mViewer);
    mViewer->setCameraPosition(3, 3, -3, 0, 0, 0);
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
inline void Drawer<PointType_>::line(std::pair<Eigen::Vector3f, Eigen::Vector3f> _line, std::string _id)
{
    auto point1 = eigenVector3fToPcl(_line.first);
    auto point2 = eigenVector3fToPcl(_line.second);
    mViewer->addLine(point1, point2, 1.0, 0.0, 0.0, _id);
    mViewer->spinOnce();
}

//---------------------------------------------------------------------------------------------------------------------
template <typename PointType_>
inline void Drawer<PointType_>::point(Eigen::Vector3f _point, std::string _id)
{
    auto point = eigenVector3fToPcl(_point);
    mViewer->addSphere(point,0.1, 0.0, 0.0, 1.0, _id,0);
    mViewer->spinOnce();
}
//---------------------------------------------------------------------------------------------------------------------
template <typename PointType_>
inline void Drawer<PointType_>::frustum(std::shared_ptr<Frustum> _frustum)
{
    // Draw frustum pose
    mViewer->addCoordinateSystem(1.0, _frustum->mPosition[0], _frustum->mPosition[1],
                                 _frustum->mPosition[2], "Frustum_" + std::to_string(_frustum->id));

    // Draw edges
    int i = 0;
    for (auto edge : _frustum->mEdges)
    {
        std::string edgeId = "Frustum_" + std::to_string(_frustum->id) + "_edge_" + std::to_string(i);
        line(edge, edgeId);
        i++;
    }

    // Draw plane normals
    Eigen::Vector3f up;
    up = _frustum->mUpPlaneNormal;
    Eigen::Vector3f right;
    right = _frustum->mRightPlaneNormal;
    mViewer->addLine(eigenVector3fToPcl(_frustum->mFpTopLeft),eigenVector3fToPcl(_frustum->mFpTopLeft+ up),
                                            1.0, 0.0, 0.0, "up_" + std::to_string(_frustum->id));
    mViewer->addLine(eigenVector3fToPcl(_frustum->mFpBotRight),eigenVector3fToPcl(_frustum->mFpBotRight - up),
                                            1.0, 0.0, 0.0, "down_" + std::to_string(_frustum->id));
    mViewer->addLine(eigenVector3fToPcl(_frustum->mFpTopRight),eigenVector3fToPcl(_frustum->mFpTopRight+ right),
                                            0.0, 0.0, 1.0, "right_" + std::to_string(_frustum->id));
    mViewer->addLine(eigenVector3fToPcl(_frustum->mFpTopLeft),eigenVector3fToPcl(_frustum->mFpTopLeft - right),
                                            0.0, 0.0, 1.0, "left_" + std::to_string(_frustum->id));
    mViewer->addLine(eigenVector3fToPcl(_frustum->mFpCenter),eigenVector3fToPcl(_frustum->mFpCenter+ _frustum->mFplaneNormal),
                                            0.0, 1.0, 0.0, "far_" + std::to_string(_frustum->id));
    mViewer->addLine(eigenVector3fToPcl(_frustum->mNpCenter),eigenVector3fToPcl(_frustum->mNpCenter+ _frustum->mNplaneNormal),
                                            0.0, 1.0, 0.0, "near_" + std::to_string(_frustum->id));                            
    // std::vector<Eigen::Vector3f> farPlane;
    // farPlane.push_back(_frustum->mFpTopLeft);
    // farPlane.push_back(_frustum->mFpTopRight);
    // farPlane.push_back(_frustum->mFpBotLeft);
    // farPlane.push_back(_frustum->mFpBotRight);

    // std::vector<Eigen::Vector3f> nearPlane;
    // nearPlane.push_back(_frustum->mNpTopLeft);
    // nearPlane.push_back(_frustum->mNpTopRight);
    // nearPlane.push_back(_frustum->mNpBotLeft);
    // nearPlane.push_back(_frustum->mNpBotRight);

    // plane(1, _frustum->mFplane, farPlane);

    // plane(2, _frustum->mNplane, nearPlane);
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
    // std::vector<pcl::Vertices> planeVertex;
    // pcl::Vertices v;
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr planePoints= typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());;
    // unsigned int i = 0;
    // for (auto point : _points)
    // {
    //     planePoints->push_back(eigenVector3fToPcl(point));
    //     v.vertices.push_back(i);
    //     i++;
    // }
    // planeVertex.push_back(v);

    // pcl::ConvexHull<pcl::PointXYZRGB> cHull;
    // cHull.setInputCloud(planePoints);

    // cHull.reconstruct(*hull,)
    // // mViewer->addPolygonMesh(planePoints,planeVertex,"PolMesh"+std::to_string(id),0);

    // mViewer->spinOnce();
}

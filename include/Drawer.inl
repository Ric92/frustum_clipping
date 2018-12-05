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
    mViewer->addSphere(point, 0.05, 0.0, 0.0, 1.0, _id, 0);
    mViewer->spinOnce();
}
//---------------------------------------------------------------------------------------------------------------------
template <typename PointType_>
inline void Drawer<PointType_>::polyhedron(int _id, Eigen::Vector3f _position, std::unordered_map<std::string, std::shared_ptr<Facet>> _facets)
{
    // Draw polyhedron pose
    mViewer->addCoordinateSystem(1.0, _position[0], _position[1],_position[2], "Frustum_" + std::to_string(_id));

    // Draw edges
    for (auto facet : _facets)
    {
        std::string facetId = "Poly_" + std::to_string(_id) + "_facet_" + facet.first;
        line(std::make_pair(facet.second->vertex[0], facet.second->vertex[1]), facetId + std::to_string(0));
        line(std::make_pair(facet.second->vertex[1], facet.second->vertex[2]), facetId + std::to_string(1));
        line(std::make_pair(facet.second->vertex[2], facet.second->vertex[3]), facetId + std::to_string(2));
        line(std::make_pair(facet.second->vertex[3], facet.second->vertex[0]), facetId + std::to_string(3));
        // std::cout << "Facet " + facet.first <<" plane= " << facet.second->plane << "\n";
    }

    // Draw plane normals
    Eigen::Vector3f up;
    up = _frustum->mUpPlaneNormal;
    Eigen::Vector3f down;
    down = _frustum->mDownPlaneNormal;
    Eigen::Vector3f right;
    right = _frustum->mRightPlaneNormal;
    Eigen::Vector3f left;
    left = _frustum->mLeftPlaneNormal;
    mViewer->addLine(eigenVector3fToPcl(_frustum->mFpTopLeft), eigenVector3fToPcl(_frustum->mFpTopLeft + up),
                     1.0, 0.0, 0.0, "up_" + std::to_string(_id));
    mViewer->addLine(eigenVector3fToPcl(_frustum->mFpBotRight), eigenVector3fToPcl(_frustum->mFpBotRight + down),
                     1.0, 0.0, 0.0, "down_" + std::to_string(_id));
    mViewer->addLine(eigenVector3fToPcl(_frustum->mFpTopRight), eigenVector3fToPcl(_frustum->mFpTopRight + right),
                     0.0, 0.0, 1.0, "right_" + std::to_string(_id));
    mViewer->addLine(eigenVector3fToPcl(_frustum->mFpTopLeft), eigenVector3fToPcl(_frustum->mFpTopLeft +left),
                     0.0, 0.0, 1.0, "left_" + std::to_string(_id));
    mViewer->addLine(eigenVector3fToPcl(_frustum->mFpCenter), eigenVector3fToPcl(_frustum->mFpCenter + _frustum->mFplaneNormal),
                     0.0, 1.0, 0.0, "far_" + std::to_string(_id));
    mViewer->addLine(eigenVector3fToPcl(_frustum->mNpCenter), eigenVector3fToPcl(_frustum->mNpCenter + _frustum->mNplaneNormal),
                     0.0, 1.0, 0.0, "near_" + std::to_string(_id));
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

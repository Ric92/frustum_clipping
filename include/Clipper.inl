
#include <cstdlib>
#include <ctime>

Clipper::Clipper()
{
    std::cout << "Clipper here \n";
    mDrawer = Drawer<pcl::PointXYZRGB>::get();
}

//---------------------------------------------------------------------------------------------------------------------
bool Clipper::clipSegmentPlane(Eigen::Vector4f _plane, Eigen::Vector3f _planeOrig,
                               std::pair<Eigen::Vector3f, Eigen::Vector3f> _segment, std::vector<Eigen::Vector3f> &_output)
{
    std::cout << "Clip with plane: " << _plane << "\n";
    Eigen::Vector3f segmentDirection = _segment.second - _segment.first;
    Eigen::Vector3f planeNormal = _plane.head(3);
    float denominator = planeNormal.dot(segmentDirection);

    if (denominator == 0)
    {
        // Segment is parallel to the plane
        float d = distanceToPlane(_plane, _segment.first);
        if (d == 0)
        {
            // Segment coincident with plane
        }
        else if (d < 0)
        {
            //Segment outside frustum
        }
        else
        {
            // Segment inside frustum
        }
        return false;
    }

    // float t = distanceToPlane(_plane, segmentDirection) / denominator;
    float t = -planeNormal.dot(_segment.first-_planeOrig) / denominator;
    if (t > -100 && t < 100)
    {
        auto intersection = _segment.first + t * (_segment.second - _segment.first);
        _output.push_back(intersection);
        
        mDrawer->point(intersection, "Intersection" + std::to_string(_plane[0])+ std::to_string(_plane[1])+ std::to_string(_plane[2]));
        std::cout << "Found Intersection t=" << t << " at point: " << intersection << "\n";
        return true;
    }
    return false;
}

//---------------------------------------------------------------------------------------------------------------------
float Clipper::distanceToPlane(Eigen::Vector4f _plane, Eigen::Vector3f _point)
{
    return _plane[0] * _point[0] + _plane[1] * _point[1] + _plane[2] * _point[2] + _plane[3];
}

//---------------------------------------------------------------------------------------------------------------------
void Clipper::clipSegmentFrustum(std::shared_ptr<Frustum> _frustum, std::pair<Eigen::Vector3f, Eigen::Vector3f> _segment)
{
    std::vector<Eigen::Vector3f> intersectionPoints;
    for (auto plane : _frustum->mPlanes)
    {
        std::vector<Eigen::Vector3f> newPoints;
        if (clipSegmentPlane(plane.second, plane.first, _segment, newPoints))
        {
            intersectionPoints.insert(intersectionPoints.end(), newPoints.begin(), newPoints.end());
        }
    }
    //std::cout << "All intersections " << intersectionPoints << "\n";
}

//---------------------------------------------------------------------------------------------------------------------
void Clipper::clipFrustumFrustum(std::shared_ptr<Frustum> _frustum1, std::shared_ptr<Frustum> _frustum2,
                                 std::vector<Eigen::Vector3f> &_intersectionPoints)
{

    std::vector<Eigen::Vector3f> intersectionPoints;
    for (auto edge : _frustum1->mEdges)
    {
        for (auto plane : _frustum2->mPlanes)
        {
            std::vector<Eigen::Vector3f> newPoints;
            if (clipSegmentPlane(plane.second, plane.first, edge, newPoints))
            {
                _intersectionPoints.insert(_intersectionPoints.end(), newPoints.begin(), newPoints.end());
            }
        }
    }
}

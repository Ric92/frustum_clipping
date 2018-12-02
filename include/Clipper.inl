
#include <cstdlib>
#include <ctime>

Clipper::Clipper()
{
    std::cout << "Clipper here \n";
    mDrawer = Drawer<pcl::PointXYZRGB>::get();
}

//---------------------------------------------------------------------------------------------------------------------
bool Clipper::clipSegmentPlane(std::shared_ptr<Facet> _facet, std::pair<Eigen::Vector3f, Eigen::Vector3f> _segment,
                               std::vector<Eigen::Vector3f> &_output)
{
    Eigen::Vector3f segmentDirection = _segment.second - _segment.first;
    Eigen::Vector3f planeNormal = _facet->plane.head(3);
    Eigen::Vector4f plane = _facet->plane;
    Eigen::Vector3f planeOrig = _facet->vertex[0];
    float den = planeNormal.dot(segmentDirection);

    if (den == 0)
    {
        // Segment is parallel to the plane
        float d = distanceToPlane(plane, _segment.first);
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

    float num = planeNormal.dot(_segment.first - planeOrig);
    float t = -num / den;
    if (t > 0 && t < 1)
    {
        auto intersection = _segment.first + t * (_segment.second - _segment.first);
        _output.push_back(intersection);
        // }
    }
}

//---------------------------------------------------------------------------------------------------------------------
float Clipper::distanceToPlane(Eigen::Vector4f _plane, Eigen::Vector3f _point)
{
    return _plane[0] * _point[0] + _plane[1] * _point[1] + _plane[2] * _point[2] + _plane[3];
}

//---------------------------------------------------------------------------------------------------------------------
void Clipper::clipSegmentFrustum(std::shared_ptr<Frustum> _frustum, std::pair<Eigen::Vector3f, Eigen::Vector3f> _segment, std::string _segmentName)
{
    std::vector<Eigen::Vector3f> intersectionPoints;
    for (auto facet : _frustum->mFacets)
    {
        std::vector<Eigen::Vector3f> newPoints;
        if (clipSegmentPlane(facet.second, _segment, newPoints))
        {
            //Check if points are inside frustum
            for (auto candidatePoint : newPoints)
            {
                //if (isInsidePolyhedron(_frustum->mFacets, candidatePoint))
                    intersectionPoints.push_back(candidatePoint);
            }
        }
    }
    int i = 0;
    for (auto inter : intersectionPoints)
    {
        mDrawer->point(inter, "Frustum_" + std::to_string(_frustum->id) + "_segment_" + _segmentName +
                                  "_inter_" + std::to_string(i));
        i++;
    }
}
//---------------------------------------------------------------------------------------------------------------------
bool Clipper::isInsidePolyhedron(std::unordered_map<std::string, std::shared_ptr<Facet>> _polyhedron, Eigen::Vector3f _point)
{
    for (auto facet : _polyhedron)
    {
        auto plane = facet.second->plane;
        float epsilon = _point[0] * plane[0] + _point[1] * plane[1] + _point[2] * plane[2] + plane[3];
        //std::cout << "Facet " + facet.first << " distance: " << epsilon << "\n";
        if (epsilon > 0.01 && facet.first != "far")
            //Point outside polyhedron
            return false;
    }
    return true;
}
//---------------------------------------------------------------------------------------------------------------------
void Clipper::clipFrustumFrustum(std::shared_ptr<Frustum> _frustum1, std::shared_ptr<Frustum> _frustum2,
                                 std::vector<Eigen::Vector3f> &_intersectionPoints)
{
    // Check segments of frustum 2 vs planes of frustum1
    int i;
    for (auto facet : _frustum2->mFacets)
    {
        std::vector<Eigen::Vector3f> vertex = facet.second->vertex;
        clipSegmentFrustum(_frustum1,std::make_pair(vertex[0], vertex[1]),"Frustum"+std::to_string(_frustum1->id)+"_segment_1"+std::to_string(i));
        clipSegmentFrustum(_frustum1,std::make_pair(vertex[1], vertex[2]),"Frustum"+std::to_string(_frustum1->id)+"_segment_2"+std::to_string(i));
        clipSegmentFrustum(_frustum1,std::make_pair(vertex[2], vertex[3]),"Frustum"+std::to_string(_frustum1->id)+"_segment_3"+std::to_string(i));
        clipSegmentFrustum(_frustum1,std::make_pair(vertex[3], vertex[0]),"Frustum"+std::to_string(_frustum1->id)+"_segment_4"+std::to_string(i));
        i++;
    }
}

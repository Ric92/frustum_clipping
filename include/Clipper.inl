
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

    // float t = distanceToPlane(_plane, segmentDirection) / den;
    float num = planeNormal.dot(_segment.first - planeOrig);
    float t = -num / den;
    if (t > 0 && t < 1)
    {
        auto intersection = _segment.first + t * (_segment.second - _segment.first);

        //Check bounds
        // if (intersection[0] < _facet->xmax && intersection[0] > _facet->xmin)
        //     if (intersection[1] < _facet->ymax && intersection[1] > _facet->ymin)
        //         if (intersection[2] < _facet->zmax && intersection[2] > _facet->zmin)
        //         {
        //std::cout << "Found Intersection t=" << t << " at point: " << intersection << "\n";
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
    //std::cout << "Clip with frustum " + std::to_string(_frustum->id) + "\n";
    std::vector<Eigen::Vector3f> intersectionPoints;
    for (auto facet : _frustum->mFacets)
    {
        //std::cout << "Clip with facet " + facet.first + "\n";
        std::vector<Eigen::Vector3f> newPoints;
        if (clipSegmentPlane(facet.second, _segment, newPoints))
        {
            //Check if points are inside frustum
            for (auto candidatePoint : newPoints)
            {
                bool inside = true;
                for (auto facet2 : _frustum->mFacets)
                {
                    auto plane = facet2.second->plane;
                    float epsilon = candidatePoint[0] * plane[0] + candidatePoint[1] * plane[1] + candidatePoint[2] * plane[2] + plane[3];
                    std::cout << "Face " + facet2.first << " distance: " << epsilon << "\n";
                    if (epsilon > 0.01) /*&& facet2.first != "far")*/
                    {
                        //Point outside frustum
                        inside = false;
                        break;
                    }
                }
                if (inside)
                    intersectionPoints.push_back(candidatePoint);
            }
            std::cout << "\n";
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
void Clipper::clipFrustumFrustum(std::shared_ptr<Frustum> _frustum1, std::shared_ptr<Frustum> _frustum2,
                                 std::vector<Eigen::Vector3f> &_intersectionPoints)
{
    // std::vector<Eigen::Vector3f> intersectionPoints;
    // for (auto edge : _frustum1->mEdges)
    // {
    //     for (auto plane : _frustum2->mPlanes)
    //     {
    //         std::vector<Eigen::Vector3f> newPoints;
    //         if (clipSegmentPlane(plane.second, plane.first, edge, newPoints))
    //         {
    //             _intersectionPoints.insert(_intersectionPoints.end(), newPoints.begin(), newPoints.end());
    //         }
    //     }
    // }
}

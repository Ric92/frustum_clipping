#include <cstdlib>
#include <ctime>

ConvexPolyhedron::ConvexPolyhedron()
{
}

//---------------------------------------------------------------------------------------------------------------------
bool ConvexPolyhedron::clipSegmentFacets(std::unordered_map<std::string, std::shared_ptr<Facet>> _polyhedronFacets,
                                         std::pair<Eigen::Vector3f, Eigen::Vector3f> _segment, std::vector<Eigen::Vector3f> &_output)
{
    for (auto facet : _polyhedronFacets)
    {
        Eigen::Vector3f segmentDirection = _segment.second - _segment.first;
        Eigen::Vector3f planeNormal = facet.second->plane.head(3);
        Eigen::Vector4f plane = facet.second->plane;
        Eigen::Vector3f planeOrig = facet.second->vertex[0];
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
        }
    }
}

//---------------------------------------------------------------------------------------------------------------------
void ConvexPolyhedron::clipConvexPolyhedron(std::shared_ptr<ConvexPolyhedron> _convexPolyhedron, std::vector<Eigen::Vector3f> &_intersectionPoints)
{

    auto convexPolyhedronFacets = _convexPolyhedron->getFacets();
    std::vector<Eigen::Vector3f> candidatePoints;
    // Intersections between facets of convex polyedron vs edges of actual polyhedron
    for (auto facet : mFacets)
    {
        std::vector<Eigen::Vector3f> vertex = facet.second->vertex;

        clipSegmentFacets(convexPolyhedronFacets, std::make_pair(vertex[0], vertex[1]), _intersectionPoints);
        clipSegmentFacets(convexPolyhedronFacets, std::make_pair(vertex[1], vertex[2]), _intersectionPoints);
        clipSegmentFacets(convexPolyhedronFacets, std::make_pair(vertex[2], vertex[3]), _intersectionPoints);
        clipSegmentFacets(convexPolyhedronFacets, std::make_pair(vertex[3], vertex[0]), _intersectionPoints);
        
        // Check convex polyedron vertices inside actual polyhedron
        // for (auto point : candidatePoints)
        // {
        //     if (isInsidePolyhedron(mFacets, point))
        //     {
        //         _intersectionPoints.push_back(point);
        //     }
        // }
    }

    // Intersections between actual polyhedron facets vs edges of convex polyedron
    for (auto facet : convexPolyhedronFacets)
    {
        std::vector<Eigen::Vector3f> vertex = facet.second->vertex;
        clipSegmentFacets(mFacets, std::make_pair(vertex[0], vertex[1]), _intersectionPoints);
        clipSegmentFacets(mFacets, std::make_pair(vertex[1], vertex[2]), _intersectionPoints);
        clipSegmentFacets(mFacets, std::make_pair(vertex[2], vertex[3]), _intersectionPoints);
        clipSegmentFacets(mFacets, std::make_pair(vertex[3], vertex[0]), _intersectionPoints);
        // Check convex polyedron vertices inside actual polyhedron
        // for (auto point : candidatePoints)
        // {
        //     if (isInsidePolyhedron(convexPolyhedronFacets, point))
        //     {
        //         _intersectionPoints.push_back(point);
        //     }
        // }
    }

    // Check convex polyedron vertices inside actual polyhedron
    for (auto vert : mVertices)
    {
        if (isInsidePolyhedron(convexPolyhedronFacets, vert))
        {
            _intersectionPoints.push_back(vert);
        }
    }
    // Check actual polyhedron vertices inside convex polyedron
    for (auto vert : _convexPolyhedron->getVertices())
    {
        if (isInsidePolyhedron(mFacets, vert))
        {
            _intersectionPoints.push_back(vert);
        }
    }
}

//---------------------------------------------------------------------------------------------------------------------
bool ConvexPolyhedron::isInsidePolyhedron(std::unordered_map<std::string, std::shared_ptr<Facet>> _polyhedron, Eigen::Vector3f _point)
{
    for (auto facet : _polyhedron)
    {
        auto plane = facet.second->plane;
        float dist = distanceToPlane(plane, _point);

        if (dist > 0.01) // TODO: Fixed value
        {
            //Point outside polyhedron
            return false;
        }
    }
    return true;
}

//---------------------------------------------------------------------------------------------------------------------
float ConvexPolyhedron::distanceToPlane(Eigen::Vector4f _plane, Eigen::Vector3f _point)
{
    return _plane[0] * _point[0] + _plane[1] * _point[1] + _plane[2] * _point[2] + _plane[3];
}

//---------------------------------------------------------------------------------------------------------------------
void ConvexPolyhedron::setFacets(std::unordered_map<std::string, std::shared_ptr<Facet>> _facets)
{
    mFacets = _facets;
}

//---------------------------------------------------------------------------------------------------------------------
std::unordered_map<std::string, std::shared_ptr<Facet>> ConvexPolyhedron::getFacets()
{
    return mFacets;
}
//---------------------------------------------------------------------------------------------------------------------
void ConvexPolyhedron::setVertices(std::vector<Eigen::Vector3f> _vertices)
{
    mVertices = _vertices;
}
//---------------------------------------------------------------------------------------------------------------------
std::vector<Eigen::Vector3f> ConvexPolyhedron::getVertices()
{
    return mVertices;
}
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

    // Intersections between facets of convex polyedron vs edges of actual polyhedron
    for (auto facet : mFacets)
    {
        std::vector<Eigen::Vector3f> candidatePoints;
        std::vector<Eigen::Vector3f> vertex = facet.second->vertex;
        for(int i = 0,j=1; i<vertex.size();i++,j++){
            if(j>=vertex.size())
                j=0;
            clipSegmentFacets(convexPolyhedronFacets, std::make_pair(vertex[i], vertex[j]), candidatePoints);
        }
        //Check candidate points inside convex polyhedron
        for (auto point : candidatePoints)
        {
            if (isInsidePolyhedron(convexPolyhedronFacets, point))
            {
                _intersectionPoints.push_back(point);
            }
        }
    }
    std::cout << std::endl << std::endl;
    // Intersections between actual polyhedron facets vs edges of convex polyedron
    for (auto facet : convexPolyhedronFacets)
    {
        std::vector<Eigen::Vector3f> candidatePoints;
        std::vector<Eigen::Vector3f> vertex = facet.second->vertex;
        for(int i = 0,j=1; i<vertex.size();i++,j++){
            if(j>=vertex.size())
                j=0;
            clipSegmentFacets(mFacets, std::make_pair(vertex[i], vertex[j]), candidatePoints);
        }
        //Check candidate points inside actual polyhedron
        for (auto point : candidatePoints)
        {
            if (isInsidePolyhedron(mFacets, point))
            {
                _intersectionPoints.push_back(point);
            }
        }
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
float ConvexPolyhedron::computeVolumeFromPoints(std::vector<Eigen::Vector3f> _points)
{
    // Create a Convex Hull
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ConvexHull<pcl::PointXYZ> chull;
    chull.setComputeAreaVolume(true);
    for (auto point : _points)
    {
        pcl::PointXYZ newPoint;
        newPoint.x = point[0];
        newPoint.y = point[1];
        newPoint.z = point[2];
        cloud_hull->push_back(newPoint);
    }
    chull.setInputCloud(cloud_hull);
    chull.reconstruct(*cloud_hull);
    // std::cout << "Convex hull has: " << cloud_hull->points.size() << " data points." << std::endl;

    return chull.getTotalVolume();
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
//---------------------------------------------------------------------------------------------------------------------
void ConvexPolyhedron::setVolume(float _volume)
{
    if (_volume > 0)
        mVolume = _volume;
    else
        std::cerr << " Negative volume \n";
}
//---------------------------------------------------------------------------------------------------------------------
float ConvexPolyhedron::getVolume()
{
    return mVolume;
}

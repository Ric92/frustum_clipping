#ifndef CONVEXPOLYHEDRON_H_
#define CONVEXPOLYHEDRON_H_

#include <Eigen/Eigen>
#include "Facet.h"
#include <pcl/point_types.h>
#include <pcl/surface/convex_hull.h>

class ConvexPolyhedron
{
public:
  ConvexPolyhedron();

  std::unordered_map<std::string, std::shared_ptr<Facet>> getFacets();
  void setFacets(std::unordered_map<std::string, std::shared_ptr<Facet>> _facets);

  std::vector<Eigen::Vector3f> getVertices();
  void setVertices(std::vector<Eigen::Vector3f> _vertices);

  float getVolume();
  void setVolume(float _volume);

  void clipConvexPolyhedron(std::shared_ptr<ConvexPolyhedron> _convexPolyhedron, std::vector<Eigen::Vector3f> &_intersectionPoints);

  float computeVolumeFromPoints(std::vector<Eigen::Vector3f> _points);

private:
  bool clipSegmentFacets(std::unordered_map<std::string, std::shared_ptr<Facet>> _polyhedronFacets,
                         std::pair<Eigen::Vector3f, Eigen::Vector3f> _segment, std::vector<Eigen::Vector3f> &_output);

  bool isInsidePolyhedron(std::unordered_map<std::string, std::shared_ptr<Facet>> _polyhedron, Eigen::Vector3f _point);

  float distanceToPlane(Eigen::Vector4f _plane, Eigen::Vector3f _point);

  // Facets
  std::unordered_map<std::string, std::shared_ptr<Facet>> mFacets;

  //Vertex
  std::vector<Eigen::Vector3f> mVertices;

  //Volume
  float mVolume;
  
};

#include <ConvexPolyhedron.inl>

#endif // CONVEXPOLYHEDRON_H_
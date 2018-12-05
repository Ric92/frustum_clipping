#ifndef CONVEX_POLYHEDRON_H_
#define CONVEX_POLYHEDRON_H_

#include <Eigen/Eigen>
#include "Frustum.h"
#include "Facet.h"

class ConvexPolyhedron
{
public:
  ConvexPolyhedron();

  std::unordered_map<std::string, std::shared_ptr<Facet>> getFacets();
  std::vector<Eigen::Vector3f> getVertices();

private:
  void clipConvexPolyhedron(std::shared_ptr<ConvexPolyhedron> _convexPolyhedron, std::vector<Eigen::Vector3f> &_intersectionPoints);

  bool clipSegmentFacets(std::unordered_map<std::string, std::shared_ptr<Facet>> _polyhedronFacets,
                         std::pair<Eigen::Vector3f, Eigen::Vector3f> _segment, std::vector<Eigen::Vector3f> &_output);

  bool isInsidePolyhedron(std::unordered_map<std::string, std::shared_ptr<Facet>> _polyhedron, Eigen::Vector3f _point);

  float distanceToPlane(Eigen::Vector4f _plane, Eigen::Vector3f _point);

private:
  // Facets
  std::unordered_map<std::string, std::shared_ptr<Facet>> mFacets;

  //Vertex
  std::vector<Eigen::Vector3f> mVertices;
};

#include <ConvexPolyhedron.inl>

#endif // CONVEX_POLYHEDRON_H_
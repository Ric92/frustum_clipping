#ifndef FRUSTUM_CLIPPING_CLIPPER_H_
#define FRUSTUM_CLIPPING_CLIPPER_H_

#include <Eigen/Eigen>
#include "Frustum.h"
#include "Facet.h"
#include "Drawer.h"

class Clipper
{
  public:
    Clipper();
    
    void clipFrustumFrustum(std::shared_ptr<Frustum> _frustum1, std::shared_ptr<Frustum> _frustum2,
                            std::vector<Eigen::Vector3f> &_intersectionPoints);

    bool clipSegmentPlane(std::shared_ptr<Facet> _facet, std::pair<Eigen::Vector3f, Eigen::Vector3f> _segment,
                          std::vector<Eigen::Vector3f> &_output);

    float distanceToPlane(Eigen::Vector4f _plane, Eigen::Vector3f _point);

    void clipSegmentFrustum(std::shared_ptr<Frustum> _frustum, std::pair<Eigen::Vector3f, Eigen::Vector3f> _segment,
                            std::string _segmentName);

  private:
    Drawer<pcl::PointXYZRGB> *mDrawer = nullptr;
};

#include <Clipper.inl>

#endif // FRUSTUM_CLIPPING_CLIPPER_H_
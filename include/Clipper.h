#ifndef FRUSTUM_CLIPPING_CLIPPER_H_
#define FRUSTUM_CLIPPING_CLIPPER_H_


#include <Eigen/Eigen>
class Clipper {
    public:
        Clipper();

        bool clipLinePlane(Eigen::Vector3f _planeNormal, Eigen::Vector3f _planeOrig, Eigen::Vector3f _ray, Eigen::Vector3f _rayOrigin, Eigen::Vector3f &_output);
};

#include <Clipper.inl>

#endif // FRUSTUM_CLIPPING_CLIPPER_H_
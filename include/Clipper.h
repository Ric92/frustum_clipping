#ifndef FRUSTUM_CLIPPING_CLIPPER_H_
#define FRUSTUM_CLIPPING_CLIPPER_H_


#include <Eigen/Eigen>
class Clipper {
    public:
        Clipper();

        Eigen::Vector3f line_plane(Eigen::Vector4f _plane, Eigen::Vector3f _line, Eigen::Vector3f _lineOrigin);
};

#include <Clipper.inl>

#endif // FRUSTUM_CLIPPING_CLIPPER_H_
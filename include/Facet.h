#ifndef FACET_H_
#define FACET_H_
#include <Eigen/Eigen>

struct Facet
{
  public:
    Facet(Eigen::Vector4f _plane, std::vector<Eigen::Vector3f> _vertex) : plane(_plane), vertex(_vertex){}

    Eigen::Vector4f plane;
    std::vector<Eigen::Vector3f> vertex;
};

#endif
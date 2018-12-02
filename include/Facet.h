#ifndef FACET_H_
#define FACET_H_
#include <Eigen/Eigen>

struct Facet
{
  public:
    Facet(Eigen::Vector4f _plane, std::vector<Eigen::Vector3f> _vertex) : plane(_plane), vertex(_vertex)
    {
        getBounds();
    }

    void getBounds()
    {
        xmax = vertex.front()[0];
        xmin = vertex.front()[0];
        ymax = vertex.front()[1];
        ymin = vertex.front()[1];
        zmax = vertex.front()[2];
        zmin = vertex.front()[2];
        for (auto point : vertex)
        {
            if (point[0] < xmin)
                xmin = point[0];
            else if (point[0] > xmax)
                xmax = point[0];
            if (point[1] < ymin)
                ymin = point[1];
            else if (point[1] > ymax)
                ymax = point[1];
            if (point[2] < zmin)
                zmin = point[2];
            else if (point[2] > zmax)
                zmax = point[2];
        }
        // std::cout << "xmax: " + std::to_string(xmax) + " xmin: " + std::to_string(xmin) +
        //                  " ymax: " + std::to_string(ymax) + " ymin: " + std::to_string(ymin) +
        //                  " zmax: " + std::to_string(zmax) + " zmin: " + std::to_string(zmin)
        //           << std::endl;
    }

    Eigen::Vector4f plane;
    std::vector<Eigen::Vector3f> vertex;
    float xmax, xmin;
    float ymax, ymin;
    float zmax, zmin;
};

#endif
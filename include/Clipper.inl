Clipper::Clipper()
{
    std::cout << "Clipper here \n";
}

bool Clipper::clipLinePlane(Eigen::Vector3f _planeNormal, Eigen::Vector3f _planeOrig, Eigen::Vector3f _ray,
                Eigen::Vector3f _rayOrigin, Eigen::Vector3f &_output)
{

    if (_planeNormal.dot(_ray) == 0)
    {
        return false;
    }
    float t = (_planeNormal.dot(_planeOrig)) / (_planeNormal.dot(_ray));
    _output = _rayOrigin + t * _ray.normalized();
    return true;
}

float Clipper::distanceToPlane(Eigen::Vector4f _plane, Eigen::Vector3f _point){
    return _plane[0]*_point[0] + _plane[1]*_point[1] + _plane[2]*_point[2] + _plane[3];
}

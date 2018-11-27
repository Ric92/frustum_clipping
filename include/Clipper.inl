Clipper::Clipper()
{
    std::cout << "Clipper here \n";
}

bool line_plane(Eigen::Vector3f _planeNormal, Eigen::Vector3f _planeOrig, Eigen::Vector3f _ray,
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

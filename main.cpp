#include <iostream>
#include <fstream>
#include <string>
#include <Eigen/Eigen>
#include <iomanip>
#include <sstream>
#include <vector>
#include <iterator>
#include <algorithm>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/plot.hpp>

#include "include/Clipper.h"
#include "src/Frustum.cpp"

int main(int argc,char **argv){
    std::cout << "Hello world \n";
    Clipper a;
    Eigen::Matrix4f b;
    b.setIdentity();
    Frustum c(b,45,45,1,5);
}
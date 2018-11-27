cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(frustum_clipping)

set(CMAKE_CXX_COMPILER /usr/bin/g++-5)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
link_directories(${EIGEN3_LIBRARY_DIRS})
add_definitions(${EIGEN3_DEFINITIONS})

find_package(OpenCV 3.4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

#find_package(PCL 1.8 REQUIRED)
# include_directories(${PCL_INCLUDE_DIRS})
# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})

include_directories(${PROJECT_SOURCE_DIR}/include)
link_directories(${PROJECT_SOURCE_DIR}/src)


add_executable (frustum_clipping main.cpp)
target_link_libraries (frustum_clipping ${EIGEN3_LIBRARIES})
target_link_libraries (frustum_clipping ${OpenCV_LIBRARIES})
# target_link_libraries (frustum_clipping ${PCL_LIBRARIES})
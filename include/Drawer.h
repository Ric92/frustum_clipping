#ifndef DRAWER_H_
#define DRAWER_H_

#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <chrono>
#include <thread>

#include "Frustum.h"

template <typename PointType_>
class Drawer
{
public:
  static bool init();
  static Drawer *get()
  {
    return mSingleton;
  }

  void frustum(int id, std::shared_ptr<Frustum>_frustum);
  void line();
  void spinOnce();

private:
  Drawer();

  void keycallback(const pcl::visualization::KeyboardEvent &_event, void *_data);
  PointType_ eigenVector3fToPcl(Eigen::Vector3f _vec);
private:
  boost::shared_ptr<pcl::visualization::PCLVisualizer> mViewer;
  bool mPause = false;
  static Drawer *mSingleton;
};

#include "Drawer.inl"

#endif
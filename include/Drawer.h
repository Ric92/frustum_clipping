#ifndef DRAWER_H_
#define DRAWER_H_

#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <chrono>
#include <thread>

template <typename PointType_>
class Drawer
{
public:
  static bool init();
  static Drawer *get()
  {
    return mSingleton;
  }
  void spinOnce();

private:
  Drawer();

  void keycallback(const pcl::visualization::KeyboardEvent &_event, void *_data);

private:
  boost::shared_ptr<pcl::visualization::PCLVisualizer> mViewer;
  bool mPause = false;
  static Drawer *mSingleton;
};

#include "Drawer.inl"

#endif
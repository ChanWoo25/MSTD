#ifndef MY_CLOUD_VISUALIZER_HPP__
#define MY_CLOUD_VISUALIZER_HPP__

#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/common/angles.h> // for pcl::deg2rad
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include <memory>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

template<class PointType>
class MyDebugVisualizer
{
private:
  enum class RUN_TYPE
  {
    ONLINE,
    OFFLINE
  } run_type_;

  inline
  bool isOnline() { return run_type_ == RUN_TYPE::ONLINE; }
  inline
  bool isOffline() { return run_type_ == RUN_TYPE::OFFLINE; }

  void run()
  {
    while (!viewer_->wasStopped())
    {
      viewer_->spinOnce(100);
      std::this_thread::sleep_for(100ms);
    }
  }

public:
  MyDebugVisualizer()=delete;
  MyDebugVisualizer(
    const std::string & run_type)
  {
    if (run_type == "online")
    {
      run_type_ = RUN_TYPE::ONLINE;
    }
    else if (run_type == "offline")
    {
      run_type_ = RUN_TYPE::OFFLINE;
    }
    else
    {
      std::cerr << "Wrong Run Type: " << run_type << std::endl;
      exit(1);
    }

    viewer_ = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer ("3D Viewer"));

    viewer_->setBackgroundColor (0, 0, 0);
    viewer_->addCoordinateSystem (1.0);
    viewer_->initCameraParameters ();
    viewer_->setShowFPS(true);
    viewer_->setCameraPosition(10.0, 0.0, 0.0, 0.0, 0.0, 1.0);
    viewer_->registerKeyboardCallback(keyboard_callback);

    // pcl::PointCloud<PointType>::Ptr cloud =
    //   new pcl::PointCloud<PointType>(pcl::PointCloud<PointType>());
    // viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
    // viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");

    run();
  }

  ~MyDebugVisualizer()
  {
  }

  static
  void keyboard_callback(
    const pcl::visualization::KeyboardEvent& event, void *)
  {
    if ( event.getKeyCode() && event.keyDown() ){
      std::cout << "Key : " << event.getKeyCode() << ", " << event.getKeySym() << std::endl;
    }
  }


  bool stopped() { return viewer_->wasStopped(); }

  void setCloud(
    pcl::PointCloud<PointType> cloud,
    const std::string & cloud_id)
  {
    // std::unique_lock<std::shared_mutex> lock(smtx_cloud_);
    if (isOnline())
    {
      if (init)
      {
        viewer_->addPointCloud<PointType>(cloud.makeShared(), cloud_id);
        viewer_->setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
          1, "sample cloud");
      }
      else
      {
        viewer_->updatePointCloud<PointType>(cloud.makeShared(), cloud_id);
      }
    }
  }

private:
  pcl::visualization::PCLVisualizer::Ptr viewer_;
  bool init = true;
};

#endif

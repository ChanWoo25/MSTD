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
#include <map>

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
    // auto cloud = pcl::PointCloud<PointType>().makeShared();
    // viewer_->addPointCloud<PointType>(cloud, "sample cloud");
    // viewer_->setPointCloudRenderingProperties(
    //   pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
    //   1, "sample cloud");
    // viewer_->setBackgroundColor (0, 0, 0);
    viewer_->initCameraParameters ();
    viewer_->setShowFPS(true);
    viewer_->setCameraPosition(10.0, 0.0, 0.0, 0.0, 0.0, 1.0);
    viewer_->registerKeyboardCallback(keyboardCB, (void *)this);

    viewer_->createViewPort(0.0, 0.0, 0.5, 1.0, v00);
    viewer_->createViewPort(0.5, 0.0, 1.0, 1.0, v01);
    viewer_->addCoordinateSystem (1.0, "v00", v00);
    viewer_->addCoordinateSystem (1.0, "v01", v01);
    viewer_->setBackgroundColor (0.1, 0.1, 0.1, v00);
    viewer_->setBackgroundColor (0.9, 0.9, 0.9, v01);
  }

  ~MyDebugVisualizer()
  {
  }

  void spin()
  {
    while (init && !viewer_->wasStopped())
    {
      viewer_->spinOnce(100);
      std::this_thread::sleep_for(20ms);
      if (press_key_s)
      {
        press_key_s = false;
        break;
      }
    }
  }

  static
  void keyboardCB(
    const pcl::visualization::KeyboardEvent& event,
    void * viewer_void)
  {
    auto viewer
      = static_cast<MyDebugVisualizer *>(viewer_void);
    if (event.keyDown())
    {
      if (event.getKeyCode() == 's' || event.getKeyCode() == 'S')
      {
        viewer->press_key_s = true;
      }
    }
  }

  // static
  // void mouseCB(
  //   const pcl::visualization::MouseEvent &event,
  //   void * viewer_void)
  // {
  //   auto viewer
  //     = static_cast<MyDebugVisualizer *>(viewer_void);
  //   // boost::shared_ptr<MyDebugVisualizer> viewer
  //   //   = *static_cast<boost::shared_ptr<MyDebugVisualizerr> *>(viewer_void);

  //   if (event.getButton() == pcl::visualization::MouseEvent::LeftButton &&
  //       event.getType() == pcl::visualization::MouseEvent::MouseButtonRelease)
  //     std::cout << "Left mouse button released at position (" << event.getX() << ", " << event.getY() << ")" << std::endl;
  // }

  bool stopped() { return viewer_->wasStopped(); }

  void setCloud(
    pcl::shared_ptr<pcl::PointCloud<PointType>> & cloud,
    const std::string & cloud_id)
  {
    // std::unique_lock<std::shared_mutex> lock(smtx_cloud_);
    if (isOnline())
    {
      if (!init)
      {
        viewer_->addPointCloud<PointType>(cloud, cloud_id, v00);
        viewer_->setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
          2.0, cloud_id, v00);
        init = true;
      }
      else
      {
        viewer_->updatePointCloud<PointType>(cloud, cloud_id);
        viewer_->setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
          2.0, cloud_id, v00);
      }
    }
  }

  void setRGBNormalCloud(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr & rgb_cloud,
    pcl::PointCloud<pcl::Normal>::Ptr & normal_cloud,
    const std::string & id,
    const double & point_size)
  {
    std::string corner_id = "corner" + id;
    std::string normal_id = "normal" + id;
    if (id_map.find(corner_id) != id_map.end())
    {
      viewer_->removePointCloud(corner_id, v01);
      viewer_->removePointCloud(normal_id, v01);
    }
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(rgb_cloud);
    viewer_->addPointCloud<pcl::PointXYZRGB> (rgb_cloud, rgb, corner_id, v01);
    viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, corner_id);
    viewer_->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (rgb_cloud, normal_cloud, 20, 0.2, normal_id, v01);
    viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 10.0, normal_id);
  }

  void setRGBCloud(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr & rgb_cloud)
  {
    // std::unique_lock<std::shared_mutex> lock(smtx_cloud_);
    if (isOnline())
    {
      if (!rgb_init)
      {
        viewer_->addPointCloud<pcl::PointXYZRGB>(rgb_cloud, "RGBCloud", v01);
        viewer_->setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
          3.0, "RGBCloud", v01);
        rgb_init = true;
      }
      else
      {
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(rgb_cloud);
        viewer_->updatePointCloud<pcl::PointXYZRGB>(rgb_cloud, rgb, "RGBCloud");
        viewer_->setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
          3.0, "RGBCloud", v01);
      }
    }
  }

  bool press_key_s = false;

private:
  pcl::visualization::PCLVisualizer::Ptr viewer_;
  bool init = false;
  bool rgb_init = false;
  int v00, v01; // used   viewport
  int v10, v11; // unused viewport
  std::map<std::string, bool> id_map;
};

#endif

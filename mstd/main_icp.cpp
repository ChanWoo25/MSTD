#include <CsvReader.hpp>
#include <MyDebugVisualizer.hpp>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <fmt/format.h>
#include <fmt/printf.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <fstream>
#include <string>

DEFINE_string(sequence_dir, "", "");
DEFINE_string(method, "icp", "'icp' or 'gicp'");

bool save_cam_params = false;

void keyboard_callback(
  const pcl::visualization::KeyboardEvent& event, void *)
{
  // if ( event.getKeyCode() && event.keyDown() ){

  //   std::cout << "Key : " << event.getKeyCode() << ", " << event.getKeySym() << std::endl;
  // }
  if (event.getKeySym() == "z")
  {
    save_cam_params = true;
  }
}

void readPCDFile(
  const std::string& pcd_file_path)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_path, *cloud) == -1) {
        PCL_ERROR("Couldn't read file %s\n", pcd_file_path.c_str());
        return;
    }

    std::cout << "Loaded " << cloud->width * cloud->height << " data points from " << pcd_file_path << std::endl;
}

void readPCDHeader(
  const std::string& pcd_file_path)
{
  std::ifstream pcd_file(pcd_file_path, std::ios::in);
  if (!pcd_file.is_open())
  {
    LOG(ERROR) << fmt::format(
      "Failed to open pcd file {}",
      pcd_file_path);
    return;
  }

  std::string line;
  while (std::getline(pcd_file, line))
  {
    if (line == "DATA ascii") {
        // Reached the end of the header
        break;
    }
    LOG(INFO) << line;
  }

  pcd_file.close();
}

size_t readLidarData(
  const std::string & data_fn,
  std::vector<double> & out_timestamps,
  std::vector<std::string> & out_scan_paths)
{
  CsvReader reader(data_fn, 2);
  while (true)
  {
    auto words = reader.next();
    if (words.empty()) { break; }
    double timestamp = std::stod(words[0]);
    std::string scan_path = words[1];
    out_timestamps.push_back(timestamp);
    out_scan_paths.push_back(scan_path);
  }
  CHECK(out_timestamps.size() == out_scan_paths.size());
  LOG(INFO) << fmt::format("Sequnce Path: {}", data_fn);
  LOG(INFO) << fmt::format(
    "Sequnce Length: {:.4f} ~ {:.4f} (#{} | {})",
    out_timestamps.front(), out_timestamps.back(),
    out_timestamps.size(),
    out_timestamps.back() - out_timestamps.front());
  return out_timestamps.size();
}

int main (int argc, char * argv[])
{
  google::SetVersionString("1.0.0");
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  // google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  auto data_fn = fmt::format("{}/data.csv", FLAGS_sequence_dir);
  std::vector<double> timestamps;
  std::vector<std::string> scan_paths;
  const auto N = readLidarData(data_fn, timestamps, scan_paths);

  // MyDebugVisualizer<pcl::PointXYZI> my_visualizer("online");

  for (size_t i = 0; i < N; i++)
  {
    const auto scan_fn = fmt::format(
      "{}/{}",
      FLAGS_sequence_dir, scan_paths[i]);
    // readPCDHeader(scan_fn);
    // char a = getchar();

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(scan_fn, *cloud) == -1) {
      LOG(ERROR) << fmt::format(
        "Couldn't read '{}' file.", scan_fn);
    }

    auto
    viewer_ = pcl::visualization::PCLVisualizer::Ptr(
                new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer_->setBackgroundColor (0, 0, 0);
    viewer_->addCoordinateSystem (1.0);
    viewer_->initCameraParameters ();
    viewer_->setCameraPosition(
      -8.2304, -10.5321, 3.96679,
      0.0429362, 0.0470072, 0.997971);
    viewer_->addPointCloud<pcl::PointXYZI>(cloud, "sample cloud");
    viewer_->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
      1, "sample cloud");
    viewer_->setPosition(468, 150);
    viewer_->setSize(1600, 1200);
    viewer_->registerKeyboardCallback(keyboard_callback);
    // viewer_->loadCameraParameters("./cam_params.txt");

    int v00{0}, v01{0};
    int v10{0}, v11{0};
    viewer_->createViewPort(0.0, 0.0, 0.5, 0.5, v00);
    viewer_->createViewPort(0.5, 0.0, 1.0, 0.5, v01);
    viewer_->createViewPort(0.0, 0.5, 0.5, 1.0, v10);
    viewer_->createViewPort(0.5, 0.5, 1.0, 1.0, v11);
    viewer_->setBackgroundColor (0, 0, 0, v00);
    viewer_->setBackgroundColor (0, 0, 0, v01);
    viewer_->setBackgroundColor (0, 0, 0, v10);
    viewer_->setBackgroundColor (0, 0, 0, v11);

    // viewer_->addText("Radius: 0.1", 10, 10, "v2 text", v2);
    // pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer_->addPointCloud<pcl::PointXYZI> (cloud, "sample cloud 00", v00);
    viewer_->addPointCloud<pcl::PointXYZI> (cloud, "sample cloud 01", v01);
    viewer_->addPointCloud<pcl::PointXYZI> (cloud, "sample cloud 10", v10);
    viewer_->addPointCloud<pcl::PointXYZI> (cloud, "sample cloud 11", v11);
    viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud 00");
    viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud 01");
    viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud 10");
    viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud 11");

    while (!viewer_->wasStopped())
    {
      viewer_->spinOnce(100);
      std::this_thread::sleep_for(100ms);
      if (save_cam_params)
      {
        viewer_->saveCameraParameters("./cam_params.txt");
        save_cam_params = false;
      }
    }
    viewer_->close();
    viewer_.reset();
  }

  return 0;
}

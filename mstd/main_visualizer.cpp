#include <CsvReader.hpp>
#include <MyDebugVisualizer.hpp>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <fmt/format.h>
#include <fmt/printf.h>
#include <fmt/core.h>
#include <fmt/ostream.h>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/gicp.h>

#include <fstream>
#include <string>
#include <algorithm>
#include <filesystem>

namespace fs = std::filesystem;

DEFINE_string(sequence_dir, "", "");
DEFINE_string(pose_fn, "", "");
DEFINE_string(method, "icp", "'icp' or 'gicp'");
DEFINE_string(save_dir, "", "");

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

size_t readPoseData(
  const std::string & data_fn,
  std::vector<double> & out_timestamps,
  std::vector<Eigen::Vector3d> & out_translations,
  std::vector<Eigen::Quaterniond> & out_quaternions)
{
  out_timestamps.clear();
  out_translations.clear();
  out_quaternions.clear();

  std::ifstream fin (data_fn);
  if (!fin.is_open()) { return 0U; }

  double timestamp;
  Eigen::Vector3d translation;
  Eigen::Quaterniond quaternion;
  std::string line;

  while (std::getline(fin, line))
  {
    std::istringstream iss(line);
    iss >> timestamp;
    iss >> translation(0);
    iss >> translation(1);
    iss >> translation(2);
    iss >> quaternion.w();
    iss >> quaternion.x();
    iss >> quaternion.y();
    iss >> quaternion.z();
    out_timestamps.push_back(timestamp);
    out_translations.push_back(translation);
    out_quaternions.push_back(quaternion);
  }

  CHECK(out_timestamps.size() == out_translations.size());
  CHECK(out_timestamps.size() == out_quaternions.size());
  return out_timestamps.size();
}

bool queryPoseByTimestamp(
  const std::vector<double> & timestamps,
  const std::vector<Eigen::Vector3d> & translations,
  const std::vector<Eigen::Quaterniond> & quaternions,
  const double & query_timestamp,
  Eigen::Vector3d & out_translation,
  Eigen::Quaterniond & out_quaternion)
{
  const auto N = timestamps.size();
  CHECK(N == translations.size());
  CHECK(N == quaternions.size());

  if (timestamps.back() <= query_timestamp)
  {
    return false;
  }

  size_t hi_idx
    = static_cast<size_t>(
          std::upper_bound(
            timestamps.begin(),
            timestamps.end(),
            query_timestamp)
        - timestamps.begin());
  size_t lo_idx = hi_idx - 1UL;

  double alpha = (query_timestamp    - timestamps[lo_idx])
               / (timestamps[hi_idx] - timestamps[lo_idx]);
  out_translation
    = (1.0 - alpha) * translations[lo_idx]
    + (alpha)       * translations[hi_idx];
  out_quaternion
    = quaternions[lo_idx].slerp(alpha, quaternions[hi_idx]);
  return true;
}

auto alignClouds(
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> & clouds,
  std::vector<Eigen::Matrix4d> & out_transformations,
  std::vector<double> & out_scores)
  -> pcl::PointCloud<pcl::PointXYZI>::Ptr
{
  out_transformations.clear();
  out_scores.clear();

  pcl::PointCloud<pcl::PointXYZI>::Ptr mergedCloud(new pcl::PointCloud<pcl::PointXYZI>);
  auto target_cloud = clouds[0];
    *mergedCloud = *target_cloud;  // Copy the first point cloud

  for (size_t i = 1UL; i < clouds.size(); i++)
  {
    // LOG(INFO) << fmt::format("cloud points: {}",mergedCloud->points.size());
    auto processor
      = (FLAGS_method == "icp")
      ? (pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>())
      : (pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>());
    processor.setMaxCorrespondenceDistance(1.0);
    processor.setTransformationEpsilon(0.001);
    processor.setMaximumIterations(1000);

    pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud = clouds[i];
    processor.setInputSource(source_cloud);
    processor.setInputTarget(target_cloud);

    // Align the source cloud to the target cloud
    pcl::PointCloud<pcl::PointXYZI> aligned_cloud;
    processor.align(aligned_cloud);
    // (*target_cloud) = (*target_cloud) + aligned_cloud;
    // target_cloud->points.insert(
    //   target_cloud->points.end(),
    //   aligned_cloud.points.begin(),
    //   aligned_cloud.points.end());
    mergedCloud->insert(mergedCloud->end(), aligned_cloud.begin(), aligned_cloud.end());

    Eigen::Matrix4d src2tgt = processor.getFinalTransformation().cast<double>();
    double score = processor.getFitnessScore();
    bool is_converged = processor.hasConverged();
    target_cloud = aligned_cloud.makeShared();
    out_transformations.push_back(src2tgt);
    out_scores.push_back(score);
  }

  // LOG(INFO) << fmt::format("cloud points: {}",mergedCloud->points.size());
  return mergedCloud;
}

int main (int argc, char * argv[])
{
  google::SetVersionString("1.0.0");
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  // google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

  const std::string data_fn = fmt::format("{}/data.csv", FLAGS_sequence_dir);
  const std::string pose_fn = FLAGS_pose_fn;
  std::vector<double> timestamps;
  std::vector<std::string> scan_paths;
  const auto N_SCANS = readLidarData(data_fn, timestamps, scan_paths);

  std::vector<double>  pose_timestamps;
  std::vector<Eigen::Vector3d>  pose_translations;
  std::vector<Eigen::Quaterniond> pose_quaternions;
  const auto N_POSES
    = readPoseData(
        pose_fn,
        pose_timestamps,
        pose_translations,
        pose_quaternions);

  constexpr int NUM_SCAN_PER_KEYFRAME = 10;

  std::string new_lidar_dir = fmt::format("{}/lidar", FLAGS_save_dir);
  std::string new_data_fn = fmt::format("{}/data.csv", FLAGS_save_dir);
  std::string new_pose_fn = fmt::format("{}/pose.txt", FLAGS_save_dir);
  try {
    fs::create_directories(new_lidar_dir);
  } catch(const std::exception & e) {
    LOG(ERROR) << e.what() << '\n';
  }

  std::ofstream fout_data, fout_pose;
  fout_data.open(new_data_fn);
  fout_pose.open(new_pose_fn);
  LOG(INFO) << "Opened: " << new_data_fn;
  LOG(INFO) << "Opened: " << new_pose_fn;
  if (!fout_data.is_open() || !fout_pose.is_open())
  {
    LOG(FATAL) << fmt::format(
      "Couldn't open {} or {}.",
      new_data_fn, new_pose_fn);
  }


  int key_index = 0;
  double curr_timestamp;
  Eigen::Matrix4d curr_pose = Eigen::Matrix4d::Identity();
  pcl::PointCloud<pcl::PointXYZI>::Ptr prev_aligned {nullptr};

  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> tmp_clouds;
  for (size_t i = 0; i < N_SCANS; i++)
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

    // LOG(INFO) << fmt::format("read {}", scan_fn);

    if (tmp_clouds.empty()) { curr_timestamp = timestamps[i]; }
    tmp_clouds.push_back(cloud);

    if (tmp_clouds.size() == 10U)
    {
      std::vector<Eigen::Matrix4d> transformations;
      std::vector<double> scores;
      pcl::PointCloud<pcl::PointXYZI>::Ptr aligned
        = alignClouds(tmp_clouds, transformations, scores);

      /* Save */
      auto new_scan_fn = fmt::format("{}/{:06d}.pcd", new_lidar_dir, key_index);
      pcl::io::savePCDFileASCII(new_scan_fn, (*aligned));
      LOG(INFO) << "Saved: " << new_scan_fn;
      fout_data << fmt::format("{:.6f},lidar/{:06d}.pcd", curr_timestamp, key_index) << std::endl;
      // fmt::print(fout_data, "{:.6f},lidar/{:06d}.pcd\n", curr_timestamp, key_index);
      if (prev_aligned)
      {
        auto processor
          = (FLAGS_method == "icp")
          ? (pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>())
          : (pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>());
        processor.setMaxCorrespondenceDistance(1.0);
        processor.setTransformationEpsilon(0.001);
        processor.setMaximumIterations(1000);
        processor.setInputSource(prev_aligned);
        processor.setInputTarget(aligned);
        pcl::PointCloud<pcl::PointXYZI> aligned_cloud;
        processor.align(aligned_cloud);
        Eigen::Matrix4d tf_prev2curr  = processor.getFinalTransformation().cast<double>();
        curr_pose = curr_pose * tf_prev2curr;
      }

      Eigen::Matrix3d curr_rotation = curr_pose.block<3,3>(0,0);
      Eigen::Vector3d curr_translation = curr_pose.block<3,1>(0,3).transpose();
      Eigen::Quaterniond curr_quat(curr_rotation);
      fout_pose << fmt::format(
        "{:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f}",
        curr_timestamp,
        curr_translation(0),
        curr_translation(1),
        curr_translation(2),
        curr_quat.x(),
        curr_quat.y(),
        curr_quat.z(),
        curr_quat.w()) << std::endl;

      prev_aligned = aligned;
      tmp_clouds.clear();
      key_index++;
    }
  }

  fout_data.close();
  fout_pose.close();

  return 0;
}

/* VIEWER CODE
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
*/

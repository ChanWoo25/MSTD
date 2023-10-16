#include <STDesc.h>
#include <CsvReader.hpp>
#include <MyDebugVisualizer.hpp>
#include <Scancontext/Scancontext.h>

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
DEFINE_string(result_dir, "", "");
DEFINE_bool(visualize, true, "");
DEFINE_int32(SC_CAND_NUM, 10, "");

void create_directories (
  const std::string & dir_path)
{
   try {
    fs::create_directories(dir_path);
  } catch(const std::exception & e) {
    LOG(ERROR) << e.what() << '\n';
  }
}

bool isSubstring(
  const std::string & str,
  const std::string & substr)
{
    return str.find(substr) != std::string::npos;
}

auto readScan(
  const std::string & rel_scan_fn)
  -> pcl::PointCloud<pcl::PointXYZI>::Ptr
{
  const auto scan_fn = fmt::format(
    "{}/{}", FLAGS_sequence_dir, rel_scan_fn);
  auto cloud = pcl::PointCloud<pcl::PointXYZI>().makeShared();
  if (isSubstring(scan_fn, "avia"))
  {
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(scan_fn, *cloud) == -1) {
      LOG(ERROR) << fmt::format(
        "Couldn't read '{}' file.", scan_fn);
    }
  }
  else if (isSubstring(scan_fn, "kitti"))
  {
    std::ifstream f_bin;
    f_bin.open(scan_fn, std::ifstream::in | std::ifstream::binary);
    if (!f_bin) {
      LOG(ERROR) << fmt::format(
        "Couldn't read '{}' file.", scan_fn);
    }
    f_bin.seekg(0, std::ios::end);
    const size_t num_elements = f_bin.tellg() / sizeof(float);
    std::vector<float> buf(num_elements);
    f_bin.seekg(0, std::ios::beg);
    f_bin.read(
      reinterpret_cast<char *>(
        &buf[0]),
        num_elements * sizeof(float));

    for (std::size_t i = 0; i < buf.size(); i += 4)
    {
      pcl::PointXYZI point;
      point.x = buf[i];
      point.y = buf[i + 1];
      point.z = buf[i + 2];
      point.intensity = buf[i + 3];
      cloud->push_back(point);
    }
    return cloud;
  }
  else
  {
    LOG(FATAL) << fmt::format("{} is unknown dataset!", scan_fn);
  }
  return cloud;
}

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
  out_timestamps.clear();
  out_scan_paths.clear();

  if (isSubstring(data_fn, "kitti"))
  {
    std::ifstream fin (data_fn);
    if (!fin.is_open()) { return 0U; }

    double timestamp;
    std::string scan_path;
    std::string line;

    while (std::getline(fin, line))
    {
      std::istringstream iss(line);
      iss >> timestamp;
      iss >> scan_path;
      out_timestamps.push_back(timestamp);
      out_scan_paths.push_back(scan_path);
    }
    return out_timestamps.size();
  }

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
    iss >> quaternion.x();
    iss >> quaternion.y();
    iss >> quaternion.z();
    iss >> quaternion.w();
    out_timestamps.push_back(timestamp);
    out_translations.push_back(translation);
    out_quaternions.push_back(quaternion);
  }

  CHECK (out_timestamps.size() == out_translations.size());
  CHECK (out_timestamps.size() == out_quaternions.size());
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
  CHECK(0.0 <= alpha && alpha <= 1.0);

  out_translation
    = (1.0 - alpha) * translations[lo_idx]
    + (alpha)       * translations[hi_idx];
  out_quaternion
    = quaternions[lo_idx].slerp(alpha, quaternions[hi_idx]);
  return true;
}

int main (int argc, char * argv[])
{
  google::SetVersionString("1.0.0");
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  // google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  pcl::console::setVerbosityLevel(pcl::console::L_INFO);

  /* Load Dataset */
  const std::string data_fn = fmt::format("{}/lidar_data.txt", FLAGS_sequence_dir);
  std::vector<double> timestamps;
  std::vector<std::string> scan_paths;
  const auto N_SCANS = readLidarData(data_fn, timestamps, scan_paths);

  std::string pose_fn = fmt::format("{}/pose.txt", FLAGS_sequence_dir);
  if (!FLAGS_pose_fn.empty()) {
    pose_fn = FLAGS_pose_fn;
  }
  std::vector<double> pose_timestamps;
  std::vector<Eigen::Vector3d> pose_translations;
  std::vector<Eigen::Quaterniond> pose_quaternions;
  const auto N_POSES = readPoseData(
    pose_fn,
    pose_timestamps,
    pose_translations,
    pose_quaternions);

  /* Ready to log results */
  create_directories(FLAGS_result_dir);
  create_directories(fmt::format("{}/pseudo_img", FLAGS_result_dir));
  const auto score_fn = fmt::format("{}/result.csv", FLAGS_result_dir);
  const auto ctime_fn = fmt::format("{}/consumption.csv", FLAGS_result_dir);
  std::ofstream fout_score, fout_ctime;
  fout_score.open(score_fn);
  fout_ctime.open(ctime_fn);
  if (fout_score.is_open()) { LOG(INFO) << "Opened: " << score_fn; }
  if (fout_ctime.is_open()) { LOG(INFO) << "Opened: " << ctime_fn; }
  CHECK(fout_score.is_open() && fout_ctime.is_open());

  auto cfg = readParamsYamlCpp(fmt::format("{}/config.yaml", FLAGS_result_dir));
  STDescManager * std_manager = new STDescManager(cfg);
  auto visualizer = MyDebugVisualizer<pcl::PointXYZI>("online");

  std::vector<double> descriptor_time;
  std::vector<double> querying_time;
  std::vector<double> update_time;
  int triggle_loop_num = 0;

  /* Main Loop */
  unsigned scan_cnt = 0;
  size_t keyframe_idx = 0;
  auto key_cloud = pcl::PointCloud<pcl::PointXYZI>().makeShared();
  auto rgb_cloud = pcl::PointCloud<pcl::PointXYZRGB>().makeShared();
  auto corner_cloud = pcl::PointCloud<pcl::PointXYZRGB>().makeShared();
  auto corner_normals = pcl::PointCloud<pcl::Normal>().makeShared();
  auto corner_cloud_before_nms = pcl::PointCloud<pcl::PointXYZRGB>().makeShared();
  auto corner_normals_before_nms = pcl::PointCloud<pcl::Normal>().makeShared();

  // pcl::VoxelGrid<SCPointType> downSizeFilterScancontext;
  // float filter_size = 0.25;
  // downSizeFilterScancontext.setLeafSize(filter_size, filter_size, filter_size);
  SCManager scManager;


  for (size_t i = 0; i < N_SCANS; i++)
  {
    auto cloud = readScan(scan_paths[i]);
    const auto n_points_before_ds = cloud->size();
    ++scan_cnt;
    // LOG(INFO) << fmt::format(
    //   "#{:02d} Time({:.6f}) Size({}",
    //   i, timestamps[i], cloud->size());

    down_sampling_voxel(*cloud, cfg.ds_size_);
    const auto n_points_after_ds = cloud->size();
    // LOG(INFO) << fmt::format(
    //   "#{:02d} Time({:.6f}) Size({} -> {})",
    //   i, timestamps[i], n_points_before_ds, n_points_after_ds);

    const auto & timestamp = timestamps[i];
    Eigen::Vector3d translation;
    Eigen::Quaterniond quaternion;
    bool succ = queryPoseByTimestamp(
      pose_timestamps,
      pose_translations,
      pose_quaternions,
      timestamp,
      translation,
      quaternion);
    Eigen::Matrix3d rotation = quaternion.toRotationMatrix();

    if (succ)
    {
      // LOG(INFO) << fmt::format(
      //   "t({:.4f}) | xyz({:.2f},{:.2f},{:.2f}) | quat({:.2f},{:.2f},{:.2f},{:.2f})",
      //   timestamp,
      //   translation(0),
      //   translation(1),
      //   translation(2),
      //   quaternion.x(),
      //   quaternion.y(),
      //   quaternion.z(),
      //   quaternion.w());
    }

    for (auto & point: cloud->points)
    {
      Eigen::Vector3d pv = point2vec(point);

      // const auto int_idx = static_cast<int>(std::floor(cloud.points[i].intensity / intensity_resolution));
      // if (0 <= int_idx && int_idx < 1000) {
      //   intensities[int_idx]++;
      // }
      // const auto range = std::pow(cloud.points[i].x, 2.0)
      //                   + std::pow(cloud.points[i].y, 2.0)
      //                   + std::pow(cloud.points[i].z, 2.0);
      // const auto ref = cloud.points[i].intensity / range;
      // const auto ref_idx = static_cast<int>(std::floor(ref / reflectivity_resolution));
      // if (0 <= ref_idx && ref_idx < 1000) {
      //   reflectivities[ref_idx]++;
      // }

      pv = rotation * pv + translation;
      point = vec2point(pv);
      key_cloud->points.push_back(point);
    }

    // LOG(INFO) << fmt::format(
    //   "#{:02d} Time({:.6f}) Size({})",
    //   i, timestamps[i], key_cloud->size());

    // constexpr bool create_img = false;
    // if (create_img)
    // {
    //   cv::Mat pseudo_img
    //     = std_manager->getPseudoImage(
    //         tmp_translations,
    //         tmp_rotations,
    //         raw_cloud,
    //         temp_cloud);
    //   std::stringstream ss;
    //   ss << std::fixed;
    //   ss.precision(6);
    //   ss << "/data/results/stdesc/park1_both/" << laser_time << ".png";
    //   cv::imwrite(ss.str(), pseudo_img);
    // }

    if (scan_cnt % 10U == 0)
    {
      /* 3. Descriptor Extraction */
      auto t_descriptor_begin = std::chrono::high_resolution_clock::now();
      std::vector<STDesc> stds_vec;
      scManager.makeAndSaveScancontextAndKeys(*key_cloud);
      auto t_descriptor_end = std::chrono::high_resolution_clock::now();
      descriptor_time.push_back(time_inc(t_descriptor_end, t_descriptor_begin));

      /* 4. Searching Loop */
      auto t_query_begin = std::chrono::high_resolution_clock::now();
      std::pair<int, double> search_result(-1, 0);
      std::pair<Eigen::Vector3d, Eigen::Matrix3d> loop_transform;
      loop_transform.first << 0, 0, 0;
      loop_transform.second = Eigen::Matrix3d::Identity();
      std::vector<std::pair<STDesc, STDesc>> loop_std_pair;
      if (i > cfg.skip_near_num_)
      {
        // std_manager->SearchLoop(
        //   stds_vec,
        //   search_result,
        //   loop_transform,
        //   loop_std_pair);

        auto detectResult = scManager.detectLoopClosureID(); // first: nn index, second: yaw diff
        int SCclosestHistoryFrameID = detectResult.first;
        if( SCclosestHistoryFrameID != -1 )
        {
          const int prev_node_idx = SCclosestHistoryFrameID;
          const int curr_node_idx = keyframe_idx; // because cpp starts 0 and ends n-1
          LOG(INFO) << "Loop detected! - between " << prev_node_idx << " and " << curr_node_idx;
        }
        auto t_query_end = std::chrono::high_resolution_clock::now();
        querying_time.push_back(time_inc(t_query_end, t_query_begin));

        // if (search_result.first > 0)
        // {
        //   LOG(INFO) << fmt::format(
        //     "Score({:.3f}) | Loop detected! With {}-th Keyframe. ",
        //     search_result.second, search_result.first);
        // }

        // // step3. Add descriptors to the database
        // auto t_map_update_begin = std::chrono::high_resolution_clock::now();
        // std_manager->AddSTDescs(stds_vec);
        // auto t_map_update_end = std::chrono::high_resolution_clock::now();
        // update_time.push_back(time_inc(t_map_update_end, t_map_update_begin));

        std_manager->key_cloud_vec_.push_back(key_cloud);
        // if (cfg.is_benchmark)
        // {
        //   std_manager->key_positions_.push_back(translation);
        //   std_manager->key_times_.push_back(laser_time);
        // }
        LOG(INFO) << "tmp size: " << key_cloud->size() << std::endl;
        if (FLAGS_visualize)
        {
          visualizer.setCloud(key_cloud, "sample cloud");
          // visualizer.setRGBCloud(rgb_cloud);
          // visualizer.setRGBNormalCloud(corner_cloud, corner_normals, "_curr", 15.0);
          // visualizer.setRGBNormalCloud(corner_cloud_before_nms, corner_normals_before_nms, "_befnms", 10.0);
        }

        if (search_result.first > 0
            || cfg.is_benchmark)
        {
          if (cfg.is_benchmark)
          {
            // ofile << std::fixed;
            // ofile.precision(6);
            // ofile << laser_time << ',';
            // ofile << search_result.second << ',';
            // Eigen::Vector3d query_translation = translation;
            // Eigen::Vector3d value_translation
            //   = std_manager->key_positions_[search_result.first];
            // ofile.precision(4);
            // ofile << query_translation(0) << ',';
            // ofile << query_translation(1) << ',';
            // ofile << query_translation(2) << ',';
            // ofile << value_translation(0) << ',';
            // ofile << value_translation(1) << ',';
            // ofile << value_translation(2) << '\n';
          }
          else
          {
            triggle_loop_num++;
            // pcl::toROSMsg(*std_manager->key_cloud_vec_[search_result.first],
            //               pub_cloud);
            // pub_cloud.header.frame_id = "camera_init";
            // pubMatchedCloud.publish(pub_cloud);
            // slow_loop.sleep();
            // pcl::toROSMsg(*std_manager->corner_cloud_vec_[search_result.first],
            //               pub_cloud);
            // pub_cloud.header.frame_id = "camera_init";
            // pubMatchedCorner.publish(pub_cloud);
            // publish_std_pairs(loop_std_pair, pubSTD);
          }
        }
      }

      if (FLAGS_visualize)
      {
        visualizer.spin();
      }

      ++keyframe_idx;
      key_cloud->clear();
      rgb_cloud->clear();
      corner_cloud->clear();
      corner_normals->clear();
      corner_cloud_before_nms->clear();
      corner_normals_before_nms->clear();
    }
  }


  // double mean_descriptor_time =
  //     std::accumulate(descriptor_time.begin(), descriptor_time.end(), 0) *
  //     1.0 / descriptor_time.size();
  // double mean_query_time =
  //     std::accumulate(querying_time.begin(), querying_time.end(), 0) * 1.0 /
  //     querying_time.size();
  // double mean_update_time =
  //     std::accumulate(update_time.begin(), update_time.end(), 0) * 1.0 /
  //     update_time.size();
  // std::cout << "Total key frame number:" << keyCloudInd
  //           << ", loop number:" << triggle_loop_num << std::endl;
  // std::cout << "Mean time for descriptor extraction: " << mean_descriptor_time
  //           << "ms, query: " << mean_query_time
  //           << "ms, update: " << mean_update_time << "ms, total: "
  //           << mean_descriptor_time + mean_query_time + mean_update_time
  //           << "ms" << std::endl;

  return 0;
}

#include <include/STDesc.h>
#include <include/MyDebugVisualizer.hpp>

#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/common/angles.h> // for pcl::deg2rad
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
// #include <geometry_msgs/Point.h>
// #include <visualization_msgs/MarkerArray.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <opencv2/highgui.hpp>
#include <fmt/core.h>
#include <fmt/format.h>

#include <iostream>
#include <fstream>
#include <thread>
#include <cstdlib>

using namespace std::chrono_literals;

std::string g_lidar_path {"/data/datasets/dataset_std/park_avia/park1.bag"};
std::string g_save_path {"/data/results/MSTD/test"};
std::string g_method {"icp"};
double      g_max_correspondence_distance {1.0};
int         g_max_iteration {1000};

// void read_config(
//   ros::NodeHandle & nh)
// {
//   nh.param<std::string>("lidar_path", g_lidar_path, "/data/datasets/dataset_std/park_avia/park1.bag");
//   nh.param<std::string>("save_path", g_save_path, "/data/results/MSTD/test");
//   nh.param<std::string>("method", g_method, "online");
//   nh.param<double>("max_correspondence_distance", g_max_correspondence_distance, 1.0);
//   nh.param<int>("max_iteration", g_max_iteration, 1000);

//   ROS_WARN_COND(g_method != "icp" && g_method != "gicp", "");
//   ROS_WARN_COND(g_lidar_path.empty(), "");
//   ROS_WARN_COND(g_save_path.empty(), "");
//   ROS_WARN_COND(g_max_correspondence_distance <= 0.0, "");
//   ROS_WARN_COND(g_max_iteration < 5, "");
// }

// auto simpleVis(
//   pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud)
//   -> std::shared_ptr<pcl::visualization::PCLVisualizer>
// {
//   // --------------------------------------------
//   // -----Open 3D viewer and add point cloud-----
//   // --------------------------------------------
//   auto viewer = std::make_shared<pcl::visualization::PCLVisualizer>("3D Viewer");
//   viewer->setBackgroundColor (0, 0, 0);
//   viewer->addPointCloud<pcl::PointXYZI>(cloud, "sample cloud");
//   viewer->setPointCloudRenderingProperties(
//     pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
//     1, "sample cloud");
//   viewer->addCoordinateSystem (1.0);
//   viewer->initCameraParameters ();
//   viewer->setShowFPS(true);
//   return viewer;
// }

// int findPoseIndexUsingTime(std::vector<double> &time_list, double &time) {
//   double time_inc = 10000000000;
//   int min_index = -1;
//   for (size_t i = 0; i < time_list.size(); i++) {
//     if (fabs(time_list[i] - time) < time_inc) {
//       time_inc = fabs(time_list[i] - time);
//       min_index = i;
//     }
//   }
//   if (time_inc > 0.5) {
//     std::string msg = "The timestamp between poses and point cloud is:" +
//                       std::to_string(time_inc) + "s. Please check it!";
//     ROS_ERROR_STREAM(msg.c_str());
//     std::cout << "Timestamp for point cloud:" << time << std::endl;
//     std::cout << "Timestamp for pose:" << time_list[min_index] << std::endl;
//     exit(-1);
//   }
//   return min_index;
// }

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "estimate_poses_using_icp");
  ros::NodeHandle nh;

  // ConfigSetting cfg;
  // read_parameters(nh, cfg);

  // read_config(nh);

  auto command = fmt::format("mkdir -p {}", g_save_path);
  ROS_INFO(command.c_str());
  system(command.c_str());

  ROS_INFO("Visualizer Start");
  MyDebugVisualizer<pcl::PointXYZI> my_visualizer("online");
  ROS_INFO("Visualizer Pass");

  // const auto pub_odom = [&pubOdomAftMapped](
  //   const Eigen::Vector3d & trans,
  //   const Eigen::Matrix3d & rot) {
  //     nav_msgs::Odometry odom;
  //     odom.header.frame_id = "camera_init";
  //     odom.pose.pose.position.x = trans[0];
  //     odom.pose.pose.position.y = trans[1];
  //     odom.pose.pose.position.z = trans[2];
  //     Eigen::Quaterniond q(rot);
  //     odom.pose.pose.orientation.w = q.w();
  //     odom.pose.pose.orientation.x = q.x();
  //     odom.pose.pose.orientation.y = q.y();
  //     odom.pose.pose.orientation.z = q.z();
  //     pubOdomAftMapped.publish(odom);
  //   };

  ros::Rate loop(500);
  ros::Rate slow_loop(10);
  // std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> poses_vec;
  // std::vector<double> times_vec;
  // load_pose_with_time(cfg.pose_path, poses_vec, times_vec, cfg);
  // std::cout << "Sucessfully load pose with number: " << poses_vec.size()
  //           << std::endl;

  // STDescManager * std_manager = new STDescManager(cfg);

  // size_t cloudInd = 0;
  // size_t keyCloudInd = 0;
  // pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(
  //     new pcl::PointCloud<pcl::PointXYZI>());
  // pcl::PointCloud<pcl::PointXYZI>::Ptr raw_cloud(
  //     new pcl::PointCloud<pcl::PointXYZI>());

  // std::vector<double> descriptor_time;
  // std::vector<double> querying_time;
  // std::vector<double> update_time;
  // int triggle_loop_num = 0;

  /* # Create Log File */
  // std::ofstream ofile;
  // std::string log_fn = cfg.log_dir + "/result.csv";
  // std::string consumption_log_fn = cfg.log_dir + "/consumption.csv";
  // if (cfg.is_benchmark)
  // {
  //   ofile.open(log_fn);
  //   if (ofile.is_open())
  //   {
  //     std::cout << "log file '" << log_fn << "' is opend!";
  //   }
  // }

  std::fstream file_;

  file_.open(g_lidar_path, std::ios::in);
  if (!file_) {
    std::cout << "File " << g_lidar_path << " does not exit" << std::endl;
  }
  ROS_INFO("Start to load the rosbag %s", g_lidar_path.c_str());
  rosbag::Bag bag;
  try {
    bag.open(g_lidar_path, rosbag::bagmode::Read);
  } catch (rosbag::BagException e) {
    ROS_ERROR_STREAM("LOADING BAG FAILED: " << e.what());
  }
  std::vector<std::string> types;
  types.push_back(std::string("sensor_msgs/PointCloud2"));
  rosbag::View view(bag, rosbag::TypeQuery(types));

  Eigen::Vector3d translation;
  Eigen::Matrix3d rotation;

  // ros::Publisher pub_source
  //   = nh.advertise<visualization_msgs::Marker>("source_points", 10);
  // ros::Publisher pub_target
  //   = nh.advertise<visualization_msgs::Marker>("target_points", 10);

  int marker_id_source = 0;
  int marker_id_target = 0;

  const int MAX_SCAN = 10;
  int cnt_scan = 0;
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> scans;
  double laser_time;

  BOOST_FOREACH (rosbag::MessageInstance const m, view)
  {
    if (!ros::ok()) { break; }

    /* 0. Data check & Conversion */
    sensor_msgs::PointCloud2::ConstPtr cloud_ptr =
        m.instantiate<sensor_msgs::PointCloud2>();
    pcl::PCLPointCloud2 pcl_pc;
    pcl::PointCloud<pcl::PointXYZI> cloud;
    if (cloud_ptr == nullptr) { continue; }
    pcl_conversions::toPCL(*cloud_ptr, pcl_pc);
    pcl::fromPCLPointCloud2(pcl_pc, cloud);
    ROS_INFO("time: %.3fs & points: %d", laser_time, cloud.points.size());

    if (scans.empty())
    {
      laser_time = cloud_ptr->header.stamp.toSec();
      // int pose_index = findPoseIndexUsingTime(times_vec, laser_time);
      // translation = poses_vec[pose_index].first;
      // rotation = poses_vec[pose_index].second;
    }

    // pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    // scan_ptr->swap(cloud);
    auto scan_ptr = cloud.makeShared();
    scans.push_back(scan_ptr); cnt_scan++;

    if (cnt_scan == MAX_SCAN)
    {
      pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud = scans[0];  // Set the first point cloud as the target
      for (size_t i = 1; i < MAX_SCAN; ++i)
      {
        // Set the source cloud for the current iteration
        pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud = scans[i];

        // Initialize ICP object
        pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
        icp.setInputSource(source_cloud);
        icp.setInputTarget(target_cloud);

        // Set ICP parameters (optional)
        icp.setMaxCorrespondenceDistance(0.1);  // Max distance for a correspondence to be valid
        icp.setMaximumIterations(50);           // Maximum iterations of the ICP algorithm

        // Align the source cloud to the target cloud
        pcl::PointCloud<pcl::PointXYZI> aligned_cloud;
        icp.align(aligned_cloud);

        if (icp.hasConverged())
        {
          std::cout << "ICP converged for pair " << i << " with score: " << icp.getFitnessScore() << std::endl;
          std::cout << "Transformation matrix:" << std::endl;
          std::cout << icp.getFinalTransformation() << std::endl;

          // Update the target cloud for the next iteration
          target_cloud = aligned_cloud.makeShared();  // Update the target for the next iteration
        }
        else
        {
            std::cerr << "ICP did not converge for pair " << i << std::endl;
            // Handle the case where ICP did not converge
        }

        my_visualizer.setCloud(*target_cloud, "sample");
      }


      // auto viewer = simpleVis(target_cloud);
      // while (!viewer->wasStopped())
      // {
      //   viewer->spinOnce(100);
      //   std::this_thread::sleep_for(100ms);
      //   /* code */
      // }

      /* Publish Two Clouds */
      // visualization_msgs::Marker marker_source;
      // marker_source.header.frame_id = "camera_init";
      // marker_source.header.stamp.fromSec(laser_time);
      // marker_source.id = 0;
      // // marker_source.id = marker_id_source++;
      // marker_source.type = visualization_msgs::Marker::POINTS;  // Marker type
      // marker_source.action = visualization_msgs::Marker::ADD;
      // marker_source.scale.x = 0.01;  // Scale
      // marker_source.scale.y = 0.01;
      // marker_source.scale.z = 0.01;
      // marker_source.color.r = 0.8;  // Color
      // marker_source.color.g = 0.0;
      // marker_source.color.b = 0.0;
      // marker_source.color.a = 0.75;
      // for (const auto & point: target_cloud->points)
      // {
      //   Eigen::Vector3d pv = point2vec(point);
      //   // marker.action = visualization_msgs::Marker::ADD;  // Action (ADD, MODIFY, etc.)
      //   pv = rotation * pv + translation;
      //   geometry_msgs::Point pt;
      //   pt.x = pv(0); pt.y = pv(1); pt.z = pv(2);
      //   marker_source.points.push_back(pt);
      // }
      // pub_source.publish(marker_source);
      // // pub_odom(translation, rotation);


      getchar();
      loop.sleep();


      scans.clear();
      cnt_scan = 0;
    }

    // if (cnt_scan % MAX_SCAN == 0 && scans.empty())
    // {
    //   laser_time = cloud_ptr->header.stamp.toSec();
    //   int pose_index = findPoseIndexUsingTime(times_vec, laser_time);
    //   Eigen::Vector3d translation = poses_vec[pose_index].first;
    //   Eigen::Matrix3d rotation = poses_vec[pose_index].second;
    // }
    // else if (cnt_scan % MAX_SCAN == 0)


    // if (   cloud_source->points.empty()
    //     && cloud_target->points.empty())
    // {
    //   cloud_source->swap(cloud);
    //   translation_source = poses_vec[pose_index].first;
    //   rotation_source = poses_vec[pose_index].second;
    //   continue;
    // }
    // else if (cloud_target->points.empty())
    // {
    //   cloud_target->swap(cloud);
    //   translation_target = poses_vec[pose_index].first;
    //   rotation_target = poses_vec[pose_index].second;
    // }
    // else
    // {
    //   cloud_source->swap(*cloud_target);
    //   translation_source = translation_target;
    //   rotation_source = rotation_target;
    //   cloud_target->swap(cloud);
    //   translation_target = poses_vec[pose_index].first;
    //   rotation_target = poses_vec[pose_index].second;
    // }


    // /* Set ICP parameters */
    // pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
    // icp.setInputSource(cloud_source);
    // icp.setInputTarget(cloud_target);
    // icp.setMaxCorrespondenceDistance(0.1);  // Max distance for a correspondence to be valid
    // icp.setMaximumIterations(50);           // Maximum iterations of the ICP algorithm

    // pcl::PointCloud<pcl::PointXYZI> aligned_cloud;
    // icp.align(aligned_cloud);

    // if (icp.hasConverged())
    // {
    //   std::cout << "ICP converged with score: " << icp.getFitnessScore() << std::endl;
    //   std::cout << "Transformation matrix:" << std::endl;
    //   std::cout << icp.getFinalTransformation() << std::endl;
    //   Eigen::Matrix4f transform = icp.getFinalTransformation();
    // }
    // else
    // {
    //   std::cerr << "ICP did not converge" << std::endl;
    // }

    // /* Publish Two Clouds */
    // visualization_msgs::MarkerArray marker_array_source;
    // visualization_msgs::MarkerArray marker_array_target;

    // visualization_msgs::Marker marker_source;
    // marker_source.header.frame_id = "camera_init";
    // marker_source.header.stamp.fromSec(laser_time);
    // marker_source.id = 0;
    // // marker_source.id = marker_id_source++;
    // marker_source.type = visualization_msgs::Marker::POINTS;  // Marker type
    // marker_source.action = visualization_msgs::Marker::ADD;
    // marker_source.scale.x = 0.01;  // Scale
    // marker_source.scale.y = 0.01;
    // marker_source.scale.z = 0.01;
    // marker_source.color.r = 0.8;  // Color
    // marker_source.color.g = 0.0;
    // marker_source.color.b = 0.0;
    // marker_source.color.a = 0.5;
    // for (const auto & point: cloud_source->points)
    // {
    //   Eigen::Vector3d pv = point2vec(point);
    //   // marker.action = visualization_msgs::Marker::ADD;  // Action (ADD, MODIFY, etc.)
    //   pv = rotation_source * pv + translation_source;
    //   geometry_msgs::Point pt;
    //   pt.x = pv(0); pt.y = pv(1); pt.z = pv(2);
    //   marker_source.points.push_back(pt);
    // }

    // visualization_msgs::Marker marker_target;
    // marker_target.header.frame_id = "camera_init";
    // marker_target.header.stamp.fromSec(laser_time);
    // marker_target.id = 0;
    // // marker_target.id = marker_id_target++;
    // marker_target.type = visualization_msgs::Marker::POINTS;  // Marker type
    // marker_target.action = visualization_msgs::Marker::ADD;
    // marker_target.scale.x = 0.01;  // Scale
    // marker_target.scale.y = 0.01;
    // marker_target.scale.z = 0.01;
    // marker_target.color.r = 0.0;  // Color
    // marker_target.color.g = 0.8;
    // marker_target.color.b = 0.0;
    // marker_target.color.a = 0.5;
    // for (const auto & point: cloud_target->points)
    // {
    //   Eigen::Vector3d pv = point2vec(point);
    //   // marker.action = visualization_msgs::Marker::ADD;  // Action (ADD, DELETE, etc.)
    //   pv = rotation_target * pv + translation_target;
    //   geometry_msgs::Point pt;
    //   pt.x = pv(0); pt.y = pv(1); pt.z = pv(2);
    //   marker_target.points.push_back(pt);
    // }

    // pub_source.publish(marker_source);
    // pub_target.publish(marker_target);
    // pub_odom(translation_target, rotation_target);


    // getchar();
    // loop.sleep();



    // pcl::PointCloud<pcl::PointXYZI> filtered_cloud;
    // for (size_t i = 0; i < cloud.size(); i++)
    // {
    //   Eigen::Vector3d pv = point2vec(cloud.points[i]);

    //   const auto int_idx = static_cast<int>(std::floor(cloud.points[i].intensity / intensity_resolution));
    //   if (0 <= int_idx && int_idx < 1000) {
    //     intensities[int_idx]++;
    //   }
    //   const auto range = std::pow(cloud.points[i].x, 2.0)
    //                     + std::pow(cloud.points[i].y, 2.0)
    //                     + std::pow(cloud.points[i].z, 2.0);
    //   const auto ref = cloud.points[i].intensity / range;
    //   const auto ref_idx = static_cast<int>(std::floor(ref / reflectivity_resolution));
    //   if (0 <= ref_idx && ref_idx < 1000) {
    //     reflectivities[ref_idx]++;
    //   }

    //   pv = rotation * pv + translation;
    //   cloud.points[i] = vec2point(pv);
    // }
    // for (auto pv : cloud.points) {
    //   raw_cloud->points.push_back(pv);
    // }
    // down_sampling_voxel(cloud, cfg.ds_size_);
    // for (auto pv : cloud.points) {
    //   if (pv.z >= cfg.z_max) { continue; }
    //   temp_cloud->points.push_back(pv);
    // }

    // /* 2. If not keyframe, pub odom & continue */
    // cloudInd++;
    // const bool is_keyframe
    //   = (cloudInd % cfg.sub_frame_num_ == 0) &&
    //     (cloudInd != 0);
    // if (is_keyframe)
    // {
    //   keyCloudInd++;
    //   constexpr bool create_img = false;
    //   if (create_img)
    //   {
    //     cv::Mat pseudo_img
    //       = std_manager->getPseudoImage(
    //           tmp_translations,
    //           tmp_rotations,
    //           raw_cloud,
    //           temp_cloud);
    //     std::stringstream ss;
    //     ss << std::fixed;
    //     ss.precision(6);
    //     ss << "/data/results/stdesc/park1_both/" << laser_time << ".png";
    //     cv::imwrite(ss.str(), pseudo_img);
    //   }
    //   tmp_translations.clear();
    //   tmp_rotations.clear();
    // }
    // else
    // {
    //   if (cfg.is_benchmark == false)
    //   {
    //     pub_odom(translation, rotation);
    //     // nav_msgs::Odometry odom;
    //     // odom.header.frame_id = "camera_init";
    //     // odom.pose.pose.position.x = translation[0];
    //     // odom.pose.pose.position.y = translation[1];
    //     // odom.pose.pose.position.z = translation[2];
    //     // Eigen::Quaterniond q(rotation);
    //     // odom.pose.pose.orientation.w = q.w();
    //     // odom.pose.pose.orientation.x = q.x();
    //     // odom.pose.pose.orientation.y = q.y();
    //     // odom.pose.pose.orientation.z = q.z();
    //     // pubOdomAftMapped.publish(odom);
    //   }
    //   loop.sleep();
    //   continue;
    // }

    // if (cfg.is_benchmark == false)
    // {
    //   std::cout.precision(4);
    //   std::cout << std::fixed << "[Time] (" << (laser_time) << ")Key Frame id:" << keyCloudInd
    //             << ", cloud size: " << temp_cloud->size() << std::endl;
    // }

    // /* 3. Descriptor Extraction */
    // auto t_descriptor_begin = std::chrono::high_resolution_clock::now();
    // std::vector<STDesc> stds_vec;
    // std_manager->GenerateSTDescs(temp_cloud, stds_vec);
    // auto t_descriptor_end = std::chrono::high_resolution_clock::now();
    // descriptor_time.push_back(
    //     time_inc(t_descriptor_end, t_descriptor_begin));

    // /* 4. Searching Loop */
    // auto t_query_begin = std::chrono::high_resolution_clock::now();
    // std::pair<int, double> search_result(-1, 0);
    // std::pair<Eigen::Vector3d, Eigen::Matrix3d> loop_transform;
    // loop_transform.first << 0, 0, 0;
    // loop_transform.second = Eigen::Matrix3d::Identity();
    // std::vector<std::pair<STDesc, STDesc>> loop_std_pair;
    // if (keyCloudInd > cfg.skip_near_num_)
    // {
    //   std_manager->SearchLoop(
    //     stds_vec,
    //     search_result,
    //     loop_transform,
    //     loop_std_pair);
    // }

    // if (search_result.first > 0 && !cfg.is_benchmark)
    // {
    //   std::cout << "[Loop Detection] triggle loop: " << keyCloudInd
    //             << "--" << search_result.first
    //             << ", score:" << search_result.second << std::endl;
    // }
    // auto t_query_end = std::chrono::high_resolution_clock::now();
    // querying_time.push_back(time_inc(t_query_end, t_query_begin));

    // // step3. Add descriptors to the database
    // auto t_map_update_begin = std::chrono::high_resolution_clock::now();
    // std_manager->AddSTDescs(stds_vec);
    // auto t_map_update_end = std::chrono::high_resolution_clock::now();
    // update_time.push_back(time_inc(t_map_update_end, t_map_update_begin));
    // std::cout << "- descriptor extraction: "
    //           << time_inc(t_descriptor_end, t_descriptor_begin) << "ms, "
    //           << "query: " << time_inc(t_query_end, t_query_begin)
    //           << "ms, "
    //           << "update map:"
    //           << time_inc(t_map_update_end, t_map_update_begin) << "ms"
    //           << std::endl;
    // std::cout << std::endl;

    // pcl::PointCloud<pcl::PointXYZI> save_key_cloud;
    // save_key_cloud = *temp_cloud;

    // std_manager->key_cloud_vec_.push_back(save_key_cloud.makeShared());

    // if (cfg.is_benchmark)
    // {
    //   std_manager->key_positions_.push_back(translation);
    //   std_manager->key_times_.push_back(laser_time);
    // }

    // // publish
    // sensor_msgs::PointCloud2 pub_cloud;
    // if (!cfg.is_benchmark)
    // {
    //   pcl::toROSMsg(*temp_cloud, pub_cloud);
    //   pub_cloud.header.frame_id = "camera_init";
    //   pubCureentCloud.publish(pub_cloud);
    //   pcl::toROSMsg(*std_manager->corner_cloud_vec_.back(), pub_cloud);
    //   pub_cloud.header.frame_id = "camera_init";
    //   pubCurrentCorner.publish(pub_cloud);
    // }

    // if (search_result.first > 0
    //     || cfg.is_benchmark)
    // {
    //   if (cfg.is_benchmark)
    //   {
    //     ofile << std::fixed;
    //     ofile.precision(6);
    //     ofile << laser_time << ',';
    //     ofile << search_result.second << ',';
    //     Eigen::Vector3d query_translation = translation;
    //     Eigen::Vector3d value_translation
    //       = std_manager->key_positions_[search_result.first];
    //     ofile.precision(4);
    //     ofile << query_translation(0) << ',';
    //     ofile << query_translation(1) << ',';
    //     ofile << query_translation(2) << ',';
    //     ofile << value_translation(0) << ',';
    //     ofile << value_translation(1) << ',';
    //     ofile << value_translation(2) << '\n';
    //   }
    //   else
    //   {
    //     triggle_loop_num++;
    //     pcl::toROSMsg(*std_manager->key_cloud_vec_[search_result.first],
    //                   pub_cloud);
    //     pub_cloud.header.frame_id = "camera_init";
    //     pubMatchedCloud.publish(pub_cloud);
    //     slow_loop.sleep();
    //     pcl::toROSMsg(*std_manager->corner_cloud_vec_[search_result.first],
    //                   pub_cloud);
    //     pub_cloud.header.frame_id = "camera_init";
    //     pubMatchedCorner.publish(pub_cloud);
    //     publish_std_pairs(loop_std_pair, pubSTD);
    //     slow_loop.sleep();
    //   }
    // }
    // temp_cloud->clear();
    // raw_cloud->clear();

    // if (!cfg.is_benchmark)
    // {
    //   getchar();
    // }


    // if (!cfg.is_benchmark)
    // {
    //   pub_odom(translation, rotation);
    //   // getchar();
    //   // nav_msgs::Odometry odom;
    //   // odom.header.frame_id = "camera_init";
    //   // odom.pose.pose.position.x = translation[0];
    //   // odom.pose.pose.position.y = translation[1];
    //   // odom.pose.pose.position.z = translation[2];
    //   // Eigen::Quaterniond q(rotation);
    //   // odom.pose.pose.orientation.w = q.w();
    //   // odom.pose.pose.orientation.x = q.x();
    //   // odom.pose.pose.orientation.y = q.y();
    //   // odom.pose.pose.orientation.z = q.z();
    //   // pubOdomAftMapped.publish(odom);
    // }

    // loop.sleep();
  } // BOOST_FOREACH (rosbag::MessageInstance const m, view)

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

  // /* # Close Log File */
  // if (cfg.is_benchmark && ofile.is_open())
  // {
  //   ofile.close();
  //   if (!ofile.is_open())
  //   {
  //     std::cout << "log file is successfully closed!";
  //   }
  // }

  // /* # Create Time Consumption Analysis File */
  // std::ofstream time_outfile;
  // if (cfg.is_benchmark)
  // {
  //   time_outfile.open(consumption_log_fn);

  //   std::cout << "descriptor_time.size : " << descriptor_time.size() << std::endl;
  //   std::cout << "querying_time.size   : " << querying_time.size() << std::endl;
  //   std::cout << "update_time.size     : " << update_time.size() << std::endl;

  //   if (time_outfile.is_open())
  //   {
  //     std::cout << "consumption log file '" << consumption_log_fn << "' is opend!" << std::endl;
  //     for (size_t i = 0; i < descriptor_time.size(); i++)
  //     {
  //       time_outfile << std::fixed;
  //       time_outfile.precision(4);
  //       time_outfile << descriptor_time[i] << ','
  //                    << querying_time[i] << ','
  //                    << update_time[i] << '\n';
  //     }
  //   }
  //   time_outfile.close();
  // }

  /* # Create reflectivity & intensity log */
  // if (cfg.is_benchmark)
  // {
  //   std::ofstream ofile2;
  //   ofile2.open("/data/results/histogram01.csv");
  //   if (ofile2.is_open())
  //   {
  //     for (size_t i = 0; i < 1000UL; i++)
  //     {
  //       ofile2 << intensities[i] << ','
  //              << reflectivities[i] << '\n';
  //     }
  //   }
  //   ofile2.close();
  // }

  return 0;
}

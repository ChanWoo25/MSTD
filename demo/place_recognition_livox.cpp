#include <include/STDesc.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <fstream>
#include <opencv2/highgui.hpp>

int findPoseIndexUsingTime(std::vector<double> &time_list, double &time) {
  double time_inc = 10000000000;
  int min_index = -1;
  for (size_t i = 0; i < time_list.size(); i++) {
    if (fabs(time_list[i] - time) < time_inc) {
      time_inc = fabs(time_list[i] - time);
      min_index = i;
    }
  }
  if (time_inc > 0.5) {
    std::string msg = "The timestamp between poses and point cloud is:" +
                      std::to_string(time_inc) + "s. Please check it!";
    ROS_ERROR_STREAM(msg.c_str());
    std::cout << "Timestamp for point cloud:" << time << std::endl;
    std::cout << "Timestamp for pose:" << time_list[min_index] << std::endl;
    exit(-1);
  }
  return min_index;
}

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "demo_kitti");
  ros::NodeHandle nh;

  ConfigSetting cfg;
  read_parameters(nh, cfg);

  ros::Publisher pubOdomAftMapped =
      nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 10);
  ros::Publisher pubCureentCloud =
      nh.advertise<sensor_msgs::PointCloud2>("/cloud_current", 100);
  ros::Publisher pubCurrentCorner =
      nh.advertise<sensor_msgs::PointCloud2>("/cloud_key_points", 100);
  ros::Publisher pubMatchedCloud =
      nh.advertise<sensor_msgs::PointCloud2>("/cloud_matched", 100);
  ros::Publisher pubMatchedCorner =
      nh.advertise<sensor_msgs::PointCloud2>("/cloud_matched_key_points", 100);
  ros::Publisher pubSTD =
      nh.advertise<visualization_msgs::MarkerArray>("descriptor_line", 10);

  const auto pub_odom = [&pubOdomAftMapped](
    const Eigen::Vector3d & trans,
    const Eigen::Matrix3d & rot) {
      nav_msgs::Odometry odom;
      odom.header.frame_id = "camera_init";
      odom.pose.pose.position.x = trans[0];
      odom.pose.pose.position.y = trans[1];
      odom.pose.pose.position.z = trans[2];
      Eigen::Quaterniond q(rot);
      odom.pose.pose.orientation.w = q.w();
      odom.pose.pose.orientation.x = q.x();
      odom.pose.pose.orientation.y = q.y();
      odom.pose.pose.orientation.z = q.z();
      pubOdomAftMapped.publish(odom);
    };

  ros::Rate loop(500);
  ros::Rate slow_loop(10);
  std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> poses_vec;
  std::vector<double> times_vec;
  load_pose_with_time(cfg.pose_path, poses_vec, times_vec, cfg);
  std::cout << "Sucessfully load pose with number: " << poses_vec.size()
            << std::endl;

  STDescManager * std_manager = new STDescManager(cfg);

  size_t cloudInd = 0;
  size_t keyCloudInd = 0;
  pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(
      new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr raw_cloud(
      new pcl::PointCloud<pcl::PointXYZI>());

  std::vector<double> descriptor_time;
  std::vector<double> querying_time;
  std::vector<double> update_time;
  int triggle_loop_num = 0;

  /* # Create Log File */
  std::ofstream ofile;
  std::string log_fn = cfg.log_dir + "/result.csv";
  std::string consumption_log_fn = cfg.log_dir + "/consumption.csv";
  if (cfg.is_benchmark)
  {
    ofile.open(log_fn);
    if (ofile.is_open())
    {
      std::cout << "log file '" << log_fn << "' is opend!";
    }
  }

  std::fstream file_;

  file_.open(cfg.lidar_path, std::ios::in);
  if (!file_) {
    std::cout << "File " << cfg.lidar_path << " does not exit" << std::endl;
  }
  ROS_INFO("Start to load the rosbag %s", cfg.lidar_path.c_str());
  rosbag::Bag bag;
  try {
    bag.open(cfg.lidar_path, rosbag::bagmode::Read);
  } catch (rosbag::BagException e) {
    ROS_ERROR_STREAM("LOADING BAG FAILED: " << e.what());
  }
  std::vector<std::string> types;
  types.push_back(std::string("sensor_msgs/PointCloud2"));
  rosbag::View view(bag, rosbag::TypeQuery(types));

  std::vector<Eigen::Vector3d> tmp_translations;
  std::vector<Eigen::Matrix3d> tmp_rotations;

  constexpr double intensity_resolution = 1.0;
  std::array<int, 1000> intensities{0};
  constexpr double reflectivity_resolution = 0.1;
  std::array<int, 1000> reflectivities{0};

  while (ros::ok())
  {
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

      /* 1. Variables Initialization */
      double laser_time = cloud_ptr->header.stamp.toSec();
      static double init_time = -1.0;
      if (init_time < 0.0) { init_time = laser_time; }
      int pose_index = findPoseIndexUsingTime(times_vec, laser_time);
      Eigen::Vector3d translation = poses_vec[pose_index].first;
      Eigen::Matrix3d rotation = poses_vec[pose_index].second;
      tmp_translations.push_back(translation);
      tmp_rotations.push_back(rotation);

      pcl::PointCloud<pcl::PointXYZI> filtered_cloud;
      for (size_t i = 0; i < cloud.size(); i++)
      {
        Eigen::Vector3d pv = point2vec(cloud.points[i]);

        const auto int_idx = static_cast<int>(std::floor(cloud.points[i].intensity / intensity_resolution));
        if (0 <= int_idx && int_idx < 1000) {
          intensities[int_idx]++;
        }
        const auto range = std::pow(cloud.points[i].x, 2.0)
                         + std::pow(cloud.points[i].y, 2.0)
                         + std::pow(cloud.points[i].z, 2.0);
        const auto ref = cloud.points[i].intensity / range;
        const auto ref_idx = static_cast<int>(std::floor(ref / reflectivity_resolution));
        if (0 <= ref_idx && ref_idx < 1000) {
          reflectivities[ref_idx]++;
        }

        pv = rotation * pv + translation;
        cloud.points[i] = vec2point(pv);
      }
      for (auto pv : cloud.points) {
        raw_cloud->points.push_back(pv);
      }
      down_sampling_voxel(cloud, cfg.ds_size_);
      for (auto pv : cloud.points) {
        if (pv.z >= cfg.z_max) { continue; }
        temp_cloud->points.push_back(pv);
      }

      /* 2. If not keyframe, pub odom & continue */
      cloudInd++;
      const bool is_keyframe
        = (cloudInd % cfg.sub_frame_num_ == 0) &&
          (cloudInd != 0);
      if (is_keyframe)
      {
        keyCloudInd++;
        constexpr bool create_img = false;
        if (create_img)
        {
          cv::Mat pseudo_img
            = std_manager->getPseudoImage(
                tmp_translations,
                tmp_rotations,
                raw_cloud,
                temp_cloud);
          std::stringstream ss;
          ss << std::fixed;
          ss.precision(6);
          ss << "/data/results/stdesc/park1_both/" << laser_time << ".png";
          cv::imwrite(ss.str(), pseudo_img);
        }
        tmp_translations.clear();
        tmp_rotations.clear();
      }
      else
      {
        if (cfg.is_benchmark == false)
        {
          pub_odom(translation, rotation);
          // nav_msgs::Odometry odom;
          // odom.header.frame_id = "camera_init";
          // odom.pose.pose.position.x = translation[0];
          // odom.pose.pose.position.y = translation[1];
          // odom.pose.pose.position.z = translation[2];
          // Eigen::Quaterniond q(rotation);
          // odom.pose.pose.orientation.w = q.w();
          // odom.pose.pose.orientation.x = q.x();
          // odom.pose.pose.orientation.y = q.y();
          // odom.pose.pose.orientation.z = q.z();
          // pubOdomAftMapped.publish(odom);
        }
        loop.sleep();
        continue;
      }

      if (cfg.is_benchmark == false)
      {
        std::cout.precision(4);
        std::cout << std::fixed << "[Time] (" << (laser_time) << ")Key Frame id:" << keyCloudInd
                  << ", cloud size: " << temp_cloud->size() << std::endl;
      }

      /* 3. Descriptor Extraction */
      auto t_descriptor_begin = std::chrono::high_resolution_clock::now();
      std::vector<STDesc> stds_vec;
      std_manager->GenerateSTDescs(temp_cloud, stds_vec);
      auto t_descriptor_end = std::chrono::high_resolution_clock::now();
      descriptor_time.push_back(
          time_inc(t_descriptor_end, t_descriptor_begin));

      /* 4. Searching Loop */
      auto t_query_begin = std::chrono::high_resolution_clock::now();
      std::pair<int, double> search_result(-1, 0);
      std::pair<Eigen::Vector3d, Eigen::Matrix3d> loop_transform;
      loop_transform.first << 0, 0, 0;
      loop_transform.second = Eigen::Matrix3d::Identity();
      std::vector<std::pair<STDesc, STDesc>> loop_std_pair;
      if (keyCloudInd > cfg.skip_near_num_)
      {
        std_manager->SearchLoop(
          stds_vec,
          search_result,
          loop_transform,
          loop_std_pair);
      }

      if (search_result.first > 0 && !cfg.is_benchmark)
      {
        std::cout << "[Loop Detection] triggle loop: " << keyCloudInd
                  << "--" << search_result.first
                  << ", score:" << search_result.second << std::endl;
      }
      auto t_query_end = std::chrono::high_resolution_clock::now();
      querying_time.push_back(time_inc(t_query_end, t_query_begin));

      // step3. Add descriptors to the database
      auto t_map_update_begin = std::chrono::high_resolution_clock::now();
      std_manager->AddSTDescs(stds_vec);
      auto t_map_update_end = std::chrono::high_resolution_clock::now();
      update_time.push_back(time_inc(t_map_update_end, t_map_update_begin));
      std::cout << "- descriptor extraction: "
                << time_inc(t_descriptor_end, t_descriptor_begin) << "ms, "
                << "query: " << time_inc(t_query_end, t_query_begin)
                << "ms, "
                << "update map:"
                << time_inc(t_map_update_end, t_map_update_begin) << "ms"
                << std::endl;
      std::cout << std::endl;

      pcl::PointCloud<pcl::PointXYZI> save_key_cloud;
      save_key_cloud = *temp_cloud;

      std_manager->key_cloud_vec_.push_back(save_key_cloud.makeShared());

      if (cfg.is_benchmark)
      {
        std_manager->key_positions_.push_back(translation);
        std_manager->key_times_.push_back(laser_time);
      }

      // publish
      sensor_msgs::PointCloud2 pub_cloud;
      if (!cfg.is_benchmark)
      {
        pcl::toROSMsg(*temp_cloud, pub_cloud);
        pub_cloud.header.frame_id = "camera_init";
        pubCureentCloud.publish(pub_cloud);
        pcl::toROSMsg(*std_manager->corner_cloud_vec_.back(), pub_cloud);
        pub_cloud.header.frame_id = "camera_init";
        pubCurrentCorner.publish(pub_cloud);
      }

      if (search_result.first > 0
          || cfg.is_benchmark)
      {
        if (cfg.is_benchmark)
        {
          ofile << std::fixed;
          ofile.precision(6);
          ofile << laser_time << ',';
          ofile << search_result.second << ',';
          Eigen::Vector3d query_translation = translation;
          Eigen::Vector3d value_translation
            = std_manager->key_positions_[search_result.first];
          ofile.precision(4);
          ofile << query_translation(0) << ',';
          ofile << query_translation(1) << ',';
          ofile << query_translation(2) << ',';
          ofile << value_translation(0) << ',';
          ofile << value_translation(1) << ',';
          ofile << value_translation(2) << '\n';
        }
        else
        {
          triggle_loop_num++;
          pcl::toROSMsg(*std_manager->key_cloud_vec_[search_result.first],
                        pub_cloud);
          pub_cloud.header.frame_id = "camera_init";
          pubMatchedCloud.publish(pub_cloud);
          slow_loop.sleep();
          pcl::toROSMsg(*std_manager->corner_cloud_vec_[search_result.first],
                        pub_cloud);
          pub_cloud.header.frame_id = "camera_init";
          pubMatchedCorner.publish(pub_cloud);
          publish_std_pairs(loop_std_pair, pubSTD);
          slow_loop.sleep();
          getchar();
        }
      }
      temp_cloud->clear();
      raw_cloud->clear();


      if (!cfg.is_benchmark)
      {
        pub_odom(translation, rotation);
        // getchar();
        // nav_msgs::Odometry odom;
        // odom.header.frame_id = "camera_init";
        // odom.pose.pose.position.x = translation[0];
        // odom.pose.pose.position.y = translation[1];
        // odom.pose.pose.position.z = translation[2];
        // Eigen::Quaterniond q(rotation);
        // odom.pose.pose.orientation.w = q.w();
        // odom.pose.pose.orientation.x = q.x();
        // odom.pose.pose.orientation.y = q.y();
        // odom.pose.pose.orientation.z = q.z();
        // pubOdomAftMapped.publish(odom);
      }

      loop.sleep();
    }
    double mean_descriptor_time =
        std::accumulate(descriptor_time.begin(), descriptor_time.end(), 0) *
        1.0 / descriptor_time.size();
    double mean_query_time =
        std::accumulate(querying_time.begin(), querying_time.end(), 0) * 1.0 /
        querying_time.size();
    double mean_update_time =
        std::accumulate(update_time.begin(), update_time.end(), 0) * 1.0 /
        update_time.size();
    std::cout << "Total key frame number:" << keyCloudInd
              << ", loop number:" << triggle_loop_num << std::endl;
    std::cout << "Mean time for descriptor extraction: " << mean_descriptor_time
              << "ms, query: " << mean_query_time
              << "ms, update: " << mean_update_time << "ms, total: "
              << mean_descriptor_time + mean_query_time + mean_update_time
              << "ms" << std::endl;
    break;
  }

  /* # Close Log File */
  if (cfg.is_benchmark && ofile.is_open())
  {
    ofile.close();
    if (!ofile.is_open())
    {
      std::cout << "log file is successfully closed!";
    }
  }

  /* # Create Time Consumption Analysis File */
  std::ofstream time_outfile;
  if (cfg.is_benchmark)
  {
    time_outfile.open(consumption_log_fn);

    std::cout << "descriptor_time.size : " << descriptor_time.size() << std::endl;
    std::cout << "querying_time.size   : " << querying_time.size() << std::endl;
    std::cout << "update_time.size     : " << update_time.size() << std::endl;

    if (time_outfile.is_open())
    {
      std::cout << "consumption log file '" << consumption_log_fn << "' is opend!" << std::endl;
      for (size_t i = 0; i < descriptor_time.size(); i++)
      {
        time_outfile << std::fixed;
        time_outfile.precision(4);
        time_outfile << descriptor_time[i] << ','
                     << querying_time[i] << ','
                     << update_time[i] << '\n';
      }
    }
    time_outfile.close();
  }

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

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <fmt/core.h>
#include <fmt/format.h>
#include <cstdlib>

std::string g_lidar_path = "/data/datasets/dataset_std/park_avia/park2.bag";
std::string g_save_path = "/data/datasets/dataset_std/park_avia_02";


int main(int argc, char * argv[])
{
  std::cout << "g_lidar_path: " << g_lidar_path << std::endl;
  std::cout << "g_save_path: " << g_save_path << std::endl;

  auto lidar_dir = fmt::format("{}/lidar", g_save_path);
  auto data_fn = fmt::format("{}/data.csv", g_save_path);

  // ConfigSetting cfg;
  // read_parameters(nh, cfg);

  // read_config(nh);

  auto command = fmt::format("mkdir -p {}", lidar_dir);
  std::cout << command << std::endl;
  system(command.c_str());


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

  // ros::Rate loop(500);
  // ros::Rate slow_loop(10);
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
  std::ofstream ofile;
  ofile.open(data_fn);
  if (ofile.is_open())
  {
    std::cout << fmt::format(
      "data_fn: {}", data_fn) << std::endl;
  }

  rosbag::Bag bag;
  bag.open(g_lidar_path, rosbag::bagmode::Read);
  std::vector<std::string> types;
  types.push_back(std::string("sensor_msgs/PointCloud2"));
  rosbag::View view(bag, rosbag::TypeQuery(types));
  double laser_time;

  int index = 0;
  BOOST_FOREACH (rosbag::MessageInstance const m, view)
  {
    /* 0. Data check & Conversion */
    sensor_msgs::PointCloud2::ConstPtr cloud_ptr =
        m.instantiate<sensor_msgs::PointCloud2>();
    pcl::PCLPointCloud2 pcl_pc;
    pcl::PointCloud<pcl::PointXYZI> cloud;
    if (cloud_ptr == nullptr) { continue; }
    pcl_conversions::toPCL(*cloud_ptr, pcl_pc);
    pcl::fromPCLPointCloud2(pcl_pc, cloud);
    laser_time = cloud_ptr->header.stamp.toSec();
    std::cout << fmt::format("time: {:.3f}s & points: {}", laser_time, cloud.points.size()) << std::endl;

    /* Save */
    // auto scan_fn = fmt::format("{}/{:06d}.pcd", lidar_dir, index);
    // pcl::io::savePCDFileASCII(scan_fn, cloud);
    // std::cout << "Saved PCD file: " << scan_fn << std::endl;
    // ofile << fmt::format("{:.6f},lidar/{:06d}.pcd\n", laser_time, index);

    index++;
    getchar();
  } // BOOST_FOREACH (rosbag::MessageInstance const m, view)

  ofile.close();

  return 0;
}

#pragma once

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <pcl/common/io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
// #include <yaml-cpp/yaml.h>

#include <fstream>
#include <mutex>
#include <memory>
#include <sstream>
#include <stdio.h>
#include <string>
#include <unordered_map>

#define HASH_P 116101
#define MAX_N 10000000000
#define MAX_FRAME_N 20000

typedef struct ConfigSetting {
  int stop_skip_enable_               {-1};
  /* for point cloud pre-preocess*/
  double ds_size_                     {-1.};
  int    maximum_corner_num_          {-1};
  /* for key points*/
  double plane_merge_normal_thre_     {-1.};
  double plane_merge_dis_thre_        {-1.};
  double plane_detection_thre_        {-1.};
  double plane_detection_thre_second_ {-1.};
  double voxel_size_                  {-1.};
  int    voxel_init_num_              {-1};
  double proj_image_resolution_       {-1.};
  double proj_dis_min_                {-1.};
  double proj_dis_max_                {-1.};
  double corner_thre_                 {-1.};
  /* for STD */
  int    descriptor_near_num_         {-1};
  double descriptor_min_len_          {-1.};
  double descriptor_max_len_          {-1.};
  double non_max_suppression_radius_  {-1.};
  double std_side_resolution_         {-1.};
  /* for place recognition*/
  int    skip_near_num_               {-1};
  int    candidate_num_               {-1};
  int    sub_frame_num_               {-1};
  double rough_dis_threshold_         {-1.};
  double vertex_diff_threshold_       {-1.};
  double icp_threshold_               {-1.};
  double normal_threshold_            {-1.};
  double dis_threshold_               {-1.};
  /* for pseudo loop gt */
  int    valid_voxel_thres_           {-1};
  /* Debugging */
  bool is_benchmark                   {false};
  std::string log_dir                 {""};
  std::string result_dir              {""};
  std::string lidar_path              {""};
  std::string pose_path               {""};
  std::string seq_name                {""};
  std::string exid                    {""};
  bool        align                   {false};
  double      z_max                   {9999.99};
  int         seq_id                  {0};
  std::string save_pseudo_loop_gt_fn  {""};
} ConfigSetting;

// Structure for Stabel Triangle Descriptor
typedef struct STDesc {
  // the side lengths of STDesc, arranged from short to long
  Eigen::Vector3d side_length_;

  // projection angle between vertices
  Eigen::Vector3d angle_;

  Eigen::Vector3d center_;
  unsigned int frame_id_;

  // three vertexs
  Eigen::Vector3d vertex_A_;
  Eigen::Vector3d vertex_B_;
  Eigen::Vector3d vertex_C_;

  // some other inform attached to each vertex,e.g., intensity
  Eigen::Vector3d vertex_attached_;
} STDesc;

// plane structure for corner point extraction
typedef struct Plane {
  pcl::PointXYZINormal p_center_;
  Eigen::Vector3d center_;
  Eigen::Vector3d normal_;
  Eigen::Matrix3d covariance_;
  float radius_ = 0;
  float min_eigen_value_ = 1;
  float intercept_ = 0;
  int id_ = 0;
  int sub_plane_num_ = 0;
  int points_size_ = 0;
  bool is_plane_ = false;
} Plane;

typedef struct STDMatchList {
  std::vector<std::pair<STDesc, STDesc>> match_list_;
  std::pair<int, int> match_id_;
  double mean_dis_;
} STDMatchList;

class VOXEL_LOC {
public:
  int64_t x, y, z;

  VOXEL_LOC(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0)
      : x(vx), y(vy), z(vz) {}

  bool operator==(const VOXEL_LOC &other) const {
    return (x == other.x && y == other.y && z == other.z);
  }
};

// for down sample function
struct M_POINT {
  float xyz[3];
  float intensity;
  int count = 0;
};

// Hash value

template <> struct std::hash<VOXEL_LOC> {
  int64_t operator()(const VOXEL_LOC &s) const {
    using std::hash;
    using std::size_t;
    return ((((s.z) * HASH_P) % MAX_N + (s.y)) * HASH_P) % MAX_N + (s.x);
  }
};

class STDesc_LOC {
public:
  int64_t x, y, z, a, b, c;

  STDesc_LOC(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0, int64_t va = 0,
             int64_t vb = 0, int64_t vc = 0)
      : x(vx), y(vy), z(vz), a(va), b(vb), c(vc) {}

  bool operator==(const STDesc_LOC &other) const {
    // use three attributes
    return (x == other.x && y == other.y && z == other.z);
    // use six attributes
    // return (x == other.x && y == other.y && z == other.z && a == other.a &&
    //         b == other.b && c == other.c);
  }
};

template <> struct std::hash<STDesc_LOC> {
  int64_t operator()(const STDesc_LOC &s) const {
    using std::hash;
    using std::size_t;
    return ((((s.z) * HASH_P) % MAX_N + (s.y)) * HASH_P) % MAX_N + (s.x);
    // return ((((((((((s.z) * HASH_P) % MAX_N + (s.y)) * HASH_P) % MAX_N +
    //              (s.x)) *
    //             HASH_P) %
    //                MAX_N +
    //            s.a) *
    //           HASH_P) %
    //              MAX_N +
    //          s.b) *
    //         HASH_P) %
    //            MAX_N +
    //        s.c;
  }
};

// OctoTree structure for plane detection
class OctoTree {
public:
  ConfigSetting config_setting_;
  std::vector<Eigen::Vector3d> voxel_points_;
  Plane *plane_ptr_;
  int layer_;
  int octo_state_; // 0 is end of tree, 1 is not
  int merge_num_ = 0;
  bool is_project_ = false;
  std::vector<Eigen::Vector3d> proj_normal_vec_;

  // check 6 direction: x,y,z,-x,-y,-z
  bool is_check_connect_[6];
  bool connect_[6];
  OctoTree *connect_tree_[6];

  bool is_publish_ = false;
  OctoTree *leaves_[8];
  double voxel_center_[3]; // x, y, z
  float quater_length_;
  bool init_octo_;
  OctoTree(const ConfigSetting &config_setting)
      : config_setting_(config_setting) {
    voxel_points_.clear();
    octo_state_ = 0;
    layer_ = 0;
    init_octo_ = false;
    for (int i = 0; i < 8; i++) {
      leaves_[i] = nullptr;
    }
    for (int i = 0; i < 6; i++) {
      is_check_connect_[i] = false;
      connect_[i] = false;
      connect_tree_[i] = nullptr;
    }
    plane_ptr_ = new Plane;
  }

  void init_plane();
  void init_octo_tree();
};

void checkPlane(
  const std::vector<Eigen::Vector3d> & points,
  Eigen::Matrix3d & covmat,
  Eigen::Vector3d & center,
  std::array<Eigen::Vector3d, 3> & eigen_vecs,
  std::array<double, 3> & eigen_vals);

void down_sampling_voxel(pcl::PointCloud<pcl::PointXYZI> &pl_feat,
                         double voxel_size);

void load_pose_with_time(
  const std::string & pose_file,
  std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> & poses_vec,
  std::vector<double> & times_vec,
  ConfigSetting & cfg);

// void read_parameters(ros::NodeHandle &nh, ConfigSetting &config_setting);

auto readParamsYamlCpp(
  const std::string & config_fn)
  ->ConfigSetting;

double time_inc(std::chrono::_V2::system_clock::time_point &t_end,
                std::chrono::_V2::system_clock::time_point &t_begin);

pcl::PointXYZI vec2point(const Eigen::Vector3d &vec);
Eigen::Vector3d point2vec(const pcl::PointXYZI &pi);

// void publish_std_pairs(
//     const std::vector<std::pair<STDesc, STDesc>> &match_std_pairs,
//     const ros::Publisher &std_publisher);

bool attach_greater_sort(std::pair<double, int> a, std::pair<double, int> b);

struct PlaneSolver {
  PlaneSolver(Eigen::Vector3d curr_point_, Eigen::Vector3d curr_normal_,
              Eigen::Vector3d target_point_, Eigen::Vector3d target_normal_)
      : curr_point(curr_point_), curr_normal(curr_normal_),
        target_point(target_point_), target_normal(target_normal_){};
  template <typename T>
  bool operator()(const T *q, const T *t, T *residual) const {
    Eigen::Quaternion<T> q_w_curr{q[3], q[0], q[1], q[2]};
    Eigen::Matrix<T, 3, 1> t_w_curr{t[0], t[1], t[2]};
    Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()),
                              T(curr_point.z())};
    Eigen::Matrix<T, 3, 1> point_w;
    point_w = q_w_curr * cp + t_w_curr;
    Eigen::Matrix<T, 3, 1> point_target(
        T(target_point.x()), T(target_point.y()), T(target_point.z()));
    Eigen::Matrix<T, 3, 1> norm(T(target_normal.x()), T(target_normal.y()),
                                T(target_normal.z()));
    residual[0] = norm.dot(point_w - point_target);
    return true;
  }

  static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_,
                                     const Eigen::Vector3d curr_normal_,
                                     Eigen::Vector3d target_point_,
                                     Eigen::Vector3d target_normal_) {
    return (
        new ceres::AutoDiffCostFunction<PlaneSolver, 1, 4, 3>(new PlaneSolver(
            curr_point_, curr_normal_, target_point_, target_normal_)));
  }

  Eigen::Vector3d curr_point;
  Eigen::Vector3d curr_normal;
  Eigen::Vector3d target_point;
  Eigen::Vector3d target_normal;
};

// Return (overlap / cloud_a_total) ratio.
void getOverlapRatio(
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_a,
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_b,
  const double & voxel_size,
  const int & voxel_occupied_threshold,
  double & max_ratio_over_tota,
  double & max_ratio_over_unio);

class STDescManager {
public:
  STDescManager() = default;

  ConfigSetting config_setting_;

  unsigned int current_frame_id_;

  STDescManager(ConfigSetting &config_setting)
      : config_setting_(config_setting) {
    current_frame_id_ = 0;
  };

  // hash table, save all descriptors
  std::unordered_map<STDesc_LOC, std::vector<STDesc>> data_base_;

  // save all key clouds, optional
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> key_cloud_vec_;

  // save all key positions
  std::vector<Eigen::Vector3d> key_positions_;

  std::vector<double> key_times_;

  // save all corner points, optional
  std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> corner_cloud_vec_;

  // save all planes of key frame, required
  std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> plane_cloud_vec_;

  /*Three main processing functions*/

  // generate STDescs from a point cloud
  void GenerateSTDescs(pcl::PointCloud<pcl::PointXYZI>::Ptr &input_cloud,
                       std::vector<STDesc> &stds_vec);

  // generate STDescs from a point cloud with RGB update
  void GenerateSTDescsRgb(
    pcl::PointCloud<pcl::PointXYZI>::Ptr & input_cloud,
    pcl::PointCloud<pcl::PointXYZRGB> & rgb_cloud,
    pcl::PointCloud<pcl::PointXYZRGB> & corner_cloud,
    pcl::PointCloud<pcl::Normal> & corner_normals,
    pcl::PointCloud<pcl::PointXYZRGB> & corner_cloud_before_nms,
    pcl::PointCloud<pcl::Normal> & corner_normals_before_nms,
    std::vector<STDesc> &stds_vec);

  // search result <candidate_id, plane icp score>. -1 for no loop
  void SearchLoop(const std::vector<STDesc> &stds_vec,
                  std::pair<int, double> &loop_result,
                  std::pair<Eigen::Vector3d, Eigen::Matrix3d> &loop_transform,
                  std::vector<std::pair<STDesc, STDesc>> &loop_std_pair);

  // add descriptors to database
  void AddSTDescs(const std::vector<STDesc> &stds_vec);

  // Geometrical optimization by plane-to-plane ico
  void PlaneGeomrtricIcp(
      const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &source_cloud,
      const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &target_cloud,
      std::pair<Eigen::Vector3d, Eigen::Matrix3d> &transform);

  // Get pseudo image from consecutive lidar scans
  auto getPseudoImage(
    const std::vector<Eigen::Vector3d> & translations,
    const std::vector<Eigen::Matrix3d> & rotations,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr & raw_cloud,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr & down_cloud)
    -> cv::Mat;

private:
  /*Following are sub-processing functions*/

  // voxelization and plane detection
  void init_voxel_map(const pcl::PointCloud<pcl::PointXYZI>::Ptr &input_cloud,
                      std::unordered_map<VOXEL_LOC, OctoTree *> &voxel_map);

  void init_voxel_map_rgb(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &input_cloud,
    pcl::PointCloud<pcl::PointXYZRGB> & rgb_cloud,
    std::unordered_map<VOXEL_LOC, OctoTree *> &voxel_map);

  // build connection for planes
  void build_connection(std::unordered_map<VOXEL_LOC, OctoTree *> &feat_map);

  // acquire planes from voxel_map
  void getPlane(const std::unordered_map<VOXEL_LOC, OctoTree *> &voxel_map,
                pcl::PointCloud<pcl::PointXYZINormal>::Ptr &plane_cloud);

  // extract corner points from pre-build voxel map and clouds
  void
  corner_extractor(
    std::unordered_map<VOXEL_LOC, OctoTree *> & voxel_map,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr & input_cloud,
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr & corner_points);

  // extract corner points from pre-build voxel map and clouds
  void
  corner_extractor_with_before(
    std::unordered_map<VOXEL_LOC, OctoTree *> & voxel_map,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr & input_cloud,
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr & corner_points,
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr & corner_before_nms);

  void
  extract_corner(const Eigen::Vector3d &proj_center,
                 const Eigen::Vector3d proj_normal,
                 const std::vector<Eigen::Vector3d> proj_points,
                 pcl::PointCloud<pcl::PointXYZINormal>::Ptr &corner_points);

  // non maximum suppression, to control the number of corners
  void non_maxi_suppression(
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr &corner_points);

  // build STDescs from corner points.
  void
  build_stdesc(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &corner_points,
               std::vector<STDesc> &stds_vec);

  // Select a specified number of candidate frames according to the number of
  // STDesc rough matches
  void candidate_selector(const std::vector<STDesc> &stds_vec,
                          std::vector<STDMatchList> &candidate_matcher_vec);

  // Get the best candidate frame by geometry check
  void
  candidate_verify(const STDMatchList &candidate_matcher, double &verify_score,
                   std::pair<Eigen::Vector3d, Eigen::Matrix3d> &relative_pose,
                   std::vector<std::pair<STDesc, STDesc>> &sucess_match_vec);

  // Get the transform between a matched std pair
  void triangle_solver(std::pair<STDesc, STDesc> &std_pair, Eigen::Vector3d &t,
                       Eigen::Matrix3d &rot);

  // Geometrical verification by plane-to-plane icp threshold
  double plane_geometric_verify(
      const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &source_cloud,
      const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &target_cloud,
      const std::pair<Eigen::Vector3d, Eigen::Matrix3d> &transform);
}; // class STDescManager

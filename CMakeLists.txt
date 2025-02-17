cmake_minimum_required(VERSION 3.0.2)
project(std_detector)

## Compile as C++14
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 14)
SET(CMAKE_BUILD_TYPE "Release")

# find_package(catkin REQUIRED COMPONENTS
#   pcl_conversions
#   pcl_ros
#   roscpp
#   rospy
#   sensor_msgs
#   std_msgs
#   tf_conversions
# )

find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)
find_package(GTSAM REQUIRED QUIET)
find_package(OpenCV  4.2.0)

## Find Eigen3
find_package(Eigen3 REQUIRED)
message(STATUS "Found eigen3 version is ${EIGEN3_VERSION_STRING}")

find_package(fmt REQUIRED)

find_package(PkgConfig REQUIRED)

pkg_check_modules(PCL_ROS REQUIRED pcl_ros)
if (PCL_ROS_FOUND)
  message(STATUS "pcl_ros version: ${PCL_ROS_VERSION}")
  message(STATUS "* ${PCL_ROS_INCLUDE_DIRS}")
  message(STATUS "* ${PCL_ROS_LIBRARIES}\n")
endif (PCL_ROS_FOUND)

pkg_check_modules(ROSBAG REQUIRED rosbag)
if (ROSBAG_FOUND)
  message(STATUS "rosbag version: ${ROSBAG_VERSION}")
  message(STATUS "* ${ROSBAG_INCLUDE_DIRS}")
  message(STATUS "* ${ROSBAG_LIBRARIES}\n")
endif (ROSBAG_FOUND)

# # find_package(glog REQUIRED)
# find_package(gflags REQUIRED)
# message(STATUS "gflags: ${GFLAGS_INCLUDE_DIRS}")


set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

include_directories(
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${ROSBAG_INCLUDE_DIRS}
  ${PCL_ROS_INCLUDE_DIRS}
  # "/usr/include/opencv4"
)

add_executable(bag_to_pcd demo/bag_to_pcd.cpp)
target_link_libraries(bag_to_pcd
  fmt::fmt
  ${PCL_LIBRARIES}
  ${ROSBAG_LIBRARIES}
  ${PCL_ROS_LIBRARIES}
  ${OpenCV_LIBS})

# add_executable(pseudo_loop_gt_detector demo/pseudo_loop_gt_detector.cpp src/STDesc.cpp )
# target_link_libraries(pseudo_loop_gt_detector
#         ${catkin_LIBRARIES}
#         ${PCL_LIBRARIES}
#         ${OpenCV_LIBS}
#         ${CERES_LIBRARIES})

# add_executable(place_recognition_kitti demo/place_recognition_kitti.cpp src/STDesc.cpp )
# target_link_libraries(place_recognition_kitti
#         ${catkin_LIBRARIES}
#         ${PCL_LIBRARIES}
#         ${OpenCV_LIBS}
#         ${CERES_LIBRARIES})

# add_executable(place_recognition_livox demo/place_recognition_livox.cpp src/STDesc.cpp )
# target_link_libraries(place_recognition_livox
#         ${catkin_LIBRARIES}
#         ${PCL_LIBRARIES}
#         ${OpenCV_LIBS}
#         ${CERES_LIBRARIES})

# add_executable(pgo_demo demo/pgo_demo.cpp src/STDesc.cpp )
# target_link_libraries(pgo_demo
#         ${catkin_LIBRARIES}
#         ${PCL_LIBRARIES}
#         ${OpenCV_LIBS}
#         ${CERES_LIBRARIES}
#         gtsam)

# add_executable(online_demo demo/online_demo.cpp src/STDesc.cpp )
# target_link_libraries(online_demo
#         ${catkin_LIBRARIES}
#         ${PCL_LIBRARIES}
#         ${OpenCV_LIBS}
#         ${CERES_LIBRARIES}
#         gtsam)

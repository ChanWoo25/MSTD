cmake_minimum_required(VERSION 3.0.2)
project(std_detector)

## Compile as C++14
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 17)
SET(CMAKE_BUILD_TYPE "Release")

find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)
find_package(GTSAM REQUIRED QUIET)
find_package(OpenCV  4.2.0)

## Find Eigen3
find_package(Eigen3 REQUIRED)
message(STATUS "Found eigen3 version is ${EIGEN3_VERSION_STRING}\n")

find_package(fmt REQUIRED)

find_package(gflags REQUIRED)
message(STATUS "gflags version: ${GFLAGS_VERSION}")
message(STATUS "* ${GFLAGS_INCLUDE_DIRS}")
message(STATUS "* ${GFLAGS_LIBRARIES}\n")

find_package(PkgConfig REQUIRED)

pkg_check_modules(GLOG REQUIRED libglog)
if (GLOG_FOUND)
  message(STATUS "glog version: ${GLOG_VERSION}")
  message(STATUS "* ${GLOG_INCLUDE_DIRS}")
  message(STATUS "* ${GLOG_LIBRARIES}\n")
endif (GLOG_FOUND)

include_directories(
  "${CMAKE_HOME_DIRECTORY}/include"
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GLOG_INCLUDE_DIRS})

add_executable(main_icp main_icp.cpp)
target_link_libraries(main_icp
  fmt::fmt
  ${PCL_LIBRARIES}
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_LIBS}
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES})
set_target_properties(main_icp PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_HOME_DIRECTORY}/bin")

add_executable(main_std
  main_std.cpp
  src/STDesc.cpp)
target_link_libraries(main_std
  fmt::fmt
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${CERES_LIBRARIES})
set_target_properties(main_std PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_HOME_DIRECTORY}/bin")

add_executable(main_scancontext
  "main_scancontext.cpp"
  "src/STDesc.cpp"
  "src/Profiler.cpp"
  "include/Scancontext/Scancontext.cpp")
target_link_libraries(main_scancontext
  fmt::fmt
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${CERES_LIBRARIES})
set_target_properties(main_scancontext PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_HOME_DIRECTORY}/bin")



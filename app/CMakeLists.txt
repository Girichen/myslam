cmake_minimum_required(VERSION 2.8)
project(run_kitti_stereo)

add_executable(run_kitti_stereo run_kitti.cpp)
target_link_libraries(run_kitti_stereo myslam)
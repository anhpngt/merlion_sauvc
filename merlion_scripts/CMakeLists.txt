cmake_minimum_required(VERSION 2.8.3)
project(merlion_scripts)

find_package(catkin REQUIRED COMPONENTS roslaunch
  roscpp
  roslib
)
find_package(OpenCV REQUIRED)
roslaunch_add_file_check(launch)

catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES velodyne_pointcloud
  # CATKIN_DEPENDS std_msgs message_runtime
  # DEPENDS system_lib
)

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

add_executable(line_detection src/line_detection.cpp)
target_link_libraries(line_detection ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
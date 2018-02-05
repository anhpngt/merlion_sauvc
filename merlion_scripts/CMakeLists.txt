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

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

macro(add_new_node name)
  add_executable(${name} src/${name}.cpp)
  target_link_libraries(${name} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
endmacro()

add_new_node(line_detection)
add_new_node(color_hist)
add_new_node(adaptive_threshold)
add_new_node(background_extract)
add_new_node(manual_background_extract)
add_new_node(bricktiles_seg)
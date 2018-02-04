cmake_minimum_required(VERSION 2.8.3)
project(merlion_perception)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  roslaunch
  rospy
  sensor_msgs
  std_msgs
)
find_package(OpenCV REQUIRED)

roslaunch_add_file_check(launch)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES merlion_perception
#  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(gate_thresholder src/gate_thresholder.cpp)
add_executable(flare_detector src/flare_detector.cpp)

target_link_libraries(gate_thresholder ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(flare_detector ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
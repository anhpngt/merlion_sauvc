cmake_minimum_required(VERSION 2.8.3)
project(merlion_scripts)

find_package(catkin REQUIRED COMPONENTS roslaunch)

roslaunch_add_file_check(launch)
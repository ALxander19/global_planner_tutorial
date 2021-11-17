cmake_minimum_required(VERSION 2.8.3)
project(freev1_global_planner)

find_package(catkin REQUIRED COMPONENTS
  pluginlib
  roscpp
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(freev1_global_planner_lib src/freev1_planner.cpp)


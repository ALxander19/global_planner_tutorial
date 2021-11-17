cmake_minimum_required(VERSION 2.8.3)
project(free_global_planner)

find_package(catkin REQUIRED COMPONENTS
  pluginlib
  roscpp
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(free_global_planner_lib src/free_planner.cpp)


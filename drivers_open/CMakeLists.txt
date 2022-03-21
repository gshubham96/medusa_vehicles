cmake_minimum_required(VERSION 3.1)
project(drivers_open)

find_package(catkin REQUIRED COMPONENTS)
catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

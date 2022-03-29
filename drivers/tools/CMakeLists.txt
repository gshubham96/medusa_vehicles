cmake_minimum_required(VERSION 2.8.3)
project(tools)

find_package(catkin REQUIRED
 COMPONENTS
 std_msgs 
 auv_msgs
 sensor_msgs 
 mavros_msgs
 dsor_msgs
 roscpp
 medusa_gimmicks_library
 tf2
 tf2_geometry_msgs
 tf2_ros
)

catkin_package(
 CATKIN_DEPENDS
)

add_compile_options(-std=c++11) 
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(
 include/tools_ros
 include/tools_algorithms
 ${catkin_INCLUDE_DIRS}
)

add_executable(imu_to_meas src/tools_ros/ImuToMeas.cpp)
add_dependencies(imu_to_meas ${catkin_EXPORTED_TARGETS})
target_link_libraries(imu_to_meas ${catkin_LIBRARIES})

add_executable(pressure_to_meas src/tools_ros/FluidPressureToMeas.cpp)
add_dependencies(pressure_to_meas ${catkin_EXPORTED_TARGETS})
target_link_libraries(pressure_to_meas ${catkin_LIBRARIES})

add_executable(ardusub_thrust_allocation src/tools_ros/ArduSubThrustAllocation.cpp)
add_dependencies(ardusub_thrust_allocation ${catkin_EXPORTED_TARGETS})
target_link_libraries(ardusub_thrust_allocation ${catkin_LIBRARIES})

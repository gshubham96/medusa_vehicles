cmake_minimum_required(VERSION 3.1)
project(medusa_ardusub)

find_package(catkin REQUIRED 
    COMPONENTS
    dsor_msgs
    mavros_msgs
    auv_msgs
    medusa_gimmicks_library
)
catkin_package(
 CATKIN_DEPENDS
)

include_directories(
    include/
    ${catkin_INCLUDE_DIRS})

add_compile_options(-std=c++11) 
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# thrust_to_pwm
add_executable(thrust_to_pwm src/ThrustToPwm.cpp)
add_dependencies(thrust_to_pwm ${catkin_EXPORTED_TARGETS})
target_link_libraries(thrust_to_pwm ${catkin_LIBRARIES})

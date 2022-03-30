cmake_minimum_required(VERSION 3.1)
project(medusa_ardusub)

find_package(catkin REQUIRED 
    COMPONENTS
    dsor_msgs
    mavros_msgs    
)
catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

# thrust_to_pwm
add_executable(thrust_to_pwm src/ThrustToPwm.cpp)
add_dependencies(thrust_to_pwm ${catkin_EXPORTED_TARGETS})
target_link_libraries(thrust_to_pwm ${catkin_LIBRARIES})

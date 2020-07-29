cmake_minimum_required(VERSION 2.8.3)
project(agv_navigation)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
)


catkin_package(
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(laser_angular_bound_filter src/laser_angular_bound_filter.cpp)
if(sensor_msgs_EXPORTED_TARGETS)
  add_dependencies(laser_angular_bound_filter ${sensor_msgs_EXPORTED_TARGETS})
endif()
target_link_libraries(laser_angular_bound_filter ${catkin_LIBRARIES})


add_executable(initial_pose src/initial_pose.cpp)
target_link_libraries(initial_pose ${catkin_LIBRARIES})

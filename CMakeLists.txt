cmake_minimum_required(VERSION 2.8.3)
project(my_pcl_tutorial)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES my_pcl_tutorial
 CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  include
)


# add_library(example
#   src/example.cpp
# )
add_executable(example
  src/example.cpp
)
add_dependencies(example ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example
  ${catkin_LIBRARIES}
)

add_executable(merge
  src/merge.cpp
)
add_dependencies(merge ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(merge
  ${catkin_LIBRARIES}
)
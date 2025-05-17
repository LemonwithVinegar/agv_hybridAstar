cmake_minimum_required(VERSION 2.8.3)
project(test)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  custom_msgs
  geometry_msgs
)


catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
  "/usr/include/eigen3"
  include
  ${catkin_INCLUDE_DIRS}
)


set(RECORDDATA_NAME "test")
add_executable(${PROJECT_NAME}_node
    src/main.cpp
    )
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})        
target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
#    ${nutils}
    )

cmake_minimum_required(VERSION 2.8.3)
project(testlink)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
   roscpp
   geographic_msgs
   custom_msgs
   sensor_msgs
   tf
   )

include_directories(
   ${catkin_INCLUDE_DIRS}
   src
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES oxts_imu
#  CATKIN_DEPENDS roscpp
#  DEPENDS
)

add_executable(testlink_node
   src/main.cpp
   )
add_dependencies(testlink_node ${custom_msgs_EXPORTED_TARGETS})
target_link_libraries(testlink_node
 ${catkin_LIBRARIES}
)


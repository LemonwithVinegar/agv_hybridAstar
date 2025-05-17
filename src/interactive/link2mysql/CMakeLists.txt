cmake_minimum_required(VERSION 2.8.3)
project(link2mysql)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
   roscpp
   custom_msgs
   geographic_msgs
   sensor_msgs
   tf
   )

include_directories(
   ${catkin_INCLUDE_DIRS}
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES oxts_imu
#  CATKIN_DEPENDS roscpp
#  DEPENDS
)

add_executable(link2mysql_node
   src/main.cpp
)
add_dependencies(link2mysql_node ${custom_msgs_EXPORTED_TARGETS})
target_link_libraries(link2mysql_node
 ${catkin_LIBRARIES}
)


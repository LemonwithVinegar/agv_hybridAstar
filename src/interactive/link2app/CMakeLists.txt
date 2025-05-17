cmake_minimum_required(VERSION 2.8.3)
project(link2app)

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

add_executable(link2app_node
   src/main.cpp
   )
add_dependencies(link2app_node ${custom_msgs_EXPORTED_TARGETS})
target_link_libraries(link2app_node
 ${catkin_LIBRARIES}
)


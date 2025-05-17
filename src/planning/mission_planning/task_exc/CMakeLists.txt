cmake_minimum_required(VERSION 2.8.3)
project(task_exc)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
   roscpp
   custom_msgs
   geographic_msgs
   sensor_msgs
   tf
   compute
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

add_executable(task_exc_node
   src/task_server.cpp
)
add_dependencies(task_exc_node ${custom_msgs_EXPORTED_TARGETS})
target_link_libraries(task_exc_node
 ${catkin_LIBRARIES}
)


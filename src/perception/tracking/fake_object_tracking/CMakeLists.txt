cmake_minimum_required(VERSION 2.8.3)
project(fake_object_tracking)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  custom_msgs
  roscpp
  visualization_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES fake_object_tracking
#  CATKIN_DEPENDS custom_msgs roscpp visualization_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

 add_executable(${PROJECT_NAME}_node src/fake_object_publisher.cpp)

 add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
 )

add_executable(dynamic_obj_node src/dynamic_obj.cpp)

add_dependencies(dynamic_obj_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(dynamic_obj_node
  ${catkin_LIBRARIES}
)


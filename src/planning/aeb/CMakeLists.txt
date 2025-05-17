cmake_minimum_required(VERSION 2.8.3)
project(aeb)


 add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  custom_msgs
  roscpp
  tf2_ros
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES aeb
#  CATKIN_DEPENDS custom_msgs roscpp tf2
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

 add_executable(${PROJECT_NAME}_node
     src/autoemergencybraking.cpp
     src/main.cpp)

 add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
 )

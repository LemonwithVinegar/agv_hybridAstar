cmake_minimum_required(VERSION 2.8.3)
project(park)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  custom_msgs
  roscpp
)

catkin_package(
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)



 add_executable(park_node
                src/park.cpp
    )

add_dependencies(park_node
    ${catkin_EXPORTED_TARGETS})

 target_link_libraries(park_node
   ${catkin_LIBRARIES}
 )


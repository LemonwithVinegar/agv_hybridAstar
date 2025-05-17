cmake_minimum_required(VERSION 2.8.3)
project(obu)

# add_compile_options(-std=c++11)

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




 add_executable(udp_obu_node_turnlight
  src/udp_obu_node_turnlight.cpp)
add_dependencies(udp_obu_node_turnlight
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS})

 target_link_libraries(udp_obu_node_turnlight
   ${catkin_LIBRARIES}
 )


# 2021-7-7
 add_executable(fake_v2x_node
                src/fake_v2x.cpp
    )

add_dependencies(fake_v2x_node
    ${catkin_EXPORTED_TARGETS})

 target_link_libraries(fake_v2x_node
   ${catkin_LIBRARIES}
 )

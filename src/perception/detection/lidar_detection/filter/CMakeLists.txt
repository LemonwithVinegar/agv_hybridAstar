cmake_minimum_required(VERSION 2.8.3)
project(filter)
add_compile_options(-std=c++14)


find_package(catkin REQUIRED COMPONENTS
        roscpp 
        compute
        custom_msgs
        )
find_package(Eigen3 REQUIRED)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES local_map_publisher
#  CATKIN_DEPENDS compute custom_msgs pcl_ros roscpp sensor_msgs visualization_msgs
#  DEPENDS system_lib
)

include_directories( ${catkin_INCLUDE_DIRS}  ${EIGEN3_INCLUDE_DIR})

add_executable(node_filter src/main.cpp
                        )
add_dependencies(node_filter ${catkin_EXPORTED_TARGETS}) 
add_dependencies(node_filter
     ${custom_msgs_EXPORTED_TARGETS})                          
target_link_libraries(node_filter ${catkin_LIBRARIES} ${EIGEN3_LIBS} 
  -lpqxx
  -lpq
  )
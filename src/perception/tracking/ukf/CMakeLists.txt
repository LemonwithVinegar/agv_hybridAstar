cmake_minimum_required(VERSION 2.8.3)
project(ukf)
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
        roscpp 
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

add_executable(ukf_node src/ukf.cpp 
                        src/main.cpp
                        )
add_dependencies(ukf_node ${catkin_EXPORTED_TARGETS}) 
add_dependencies(ukf_node
     ${custom_msgs_EXPORTED_TARGETS})                          
target_link_libraries(ukf_node ${catkin_LIBRARIES} ${EIGEN3_LIBS} 
  )
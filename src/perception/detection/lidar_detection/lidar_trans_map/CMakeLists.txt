cmake_minimum_required(VERSION 2.8.3)
project(lidar_trans_map)
add_compile_options(-std=c++14)


find_package(catkin REQUIRED COMPONENTS
        roscpp 
        tf
        custom_msgs
        geometry_msgs
        tf2_eigen
        )

catkin_package( )

include_directories( ${catkin_INCLUDE_DIRS}  )

add_executable(lidar_to_map src/main.cpp 
                        )
add_dependencies(lidar_to_map
     ${custom_msgs_EXPORTED_TARGETS})                       
target_link_libraries(lidar_to_map ${catkin_LIBRARIES}
  )
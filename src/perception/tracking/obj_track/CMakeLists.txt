cmake_minimum_required(VERSION 2.8.3)
project(obj_track)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        compute
        custom_msgs
        )

catkin_package( )

include_directories( ${catkin_INCLUDE_DIRS}  )

add_executable(obj_track src/main.cpp 
                         src/association.cpp
                         src/kf.cpp
)
add_dependencies(obj_track ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})                       
target_link_libraries(obj_track ${catkin_LIBRARIES})
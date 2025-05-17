cmake_minimum_required(VERSION 2.8.3)
project(object_track)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        compute
        custom_msgs
        )
find_package(OpenCV REQUIRED)
catkin_package( )
include_directories( ${catkin_INCLUDE_DIRS}  ${OpenCV_INCLUDE_DIRS} )
# include_directories( ${catkin_INCLUDE_DIRS}  )

add_executable(track_obj src/main.cpp 
                         src/data_association.cpp
                        #  src/kalman.cpp
)
add_dependencies(track_obj ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})                       
target_link_libraries(track_obj ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
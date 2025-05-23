cmake_minimum_required(VERSION 2.8.3)
project(euclidean_cluster_detect)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -std=c++14")


find_package(catkin REQUIRED COMPONENTS
        roscpp 
        roslib 
        std_msgs 
        sensor_msgs 
        pcl_conversions
        pcl_ros 
        # cv_bridge     # ubuntu18 下使用
        cv_bridge_new   # ubuntu20 下使用
        image_transport 
        custom_msgs
        )
find_package(Boost REQUIRED COMPONENTS )
find_package(OpenCV REQUIRED)

catkin_package( )

include_directories( ${catkin_INCLUDE_DIRS}  ${OpenCV_INCLUDE_DIRS} )

add_executable(targetDetect src/main.cpp 
                        src/cluster.cpp
                        src/cluster.h
                        src/boxer.h
                        src/boxer.cpp
                        src/tracker.h
                        src/tracker.cpp
                        src/hungarian_bigraph_matcher.h
                        src/hungarian_bigraph_matcher.cc
                        src/tracker_object.h 
                        )
add_dependencies(targetDetect
     ${custom_msgs_EXPORTED_TARGETS})                       
target_link_libraries(targetDetect ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} 
  -lpqxx
  -lpq
  )


add_executable(lidarRawpoint_saver
    src/savetxt.cpp)
add_dependencies(lidarRawpoint_saver ${custom_msgs_EXPORTED_TARGETS})
target_link_libraries(lidarRawpoint_saver
  ${catkin_LIBRARIES}
)
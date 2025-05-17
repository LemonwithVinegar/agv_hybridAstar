cmake_minimum_required(VERSION 2.8.3)
project(ground_segmentaion)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

SET(CMAKE_CXX_FLAGS "-std=c++14 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

link_directories(${PCL_LIBRARY_DIRS})

#Ground Plane Fitter
find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)
add_definitions(${PCL_DEFINITIONS})

add_executable(myground
	           src/groundplanfit.cpp
)

target_include_directories(myground PRIVATE
	${PCL_INCLUDE_DIRS}
)

target_link_libraries(myground
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)


add_executable(points_converter
                   src/points_converter.cpp
)

target_include_directories(points_converter PRIVATE
        ${PCL_INCLUDE_DIRS}
)

target_link_libraries(points_converter
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(myground_hesai
	           src/groundplanfit_hesai.cpp
)

target_include_directories(myground_hesai PRIVATE
	${PCL_INCLUDE_DIRS}
)

target_link_libraries(myground_hesai
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(myground_leishen
	           src/groundplanfit_leishen.cpp
)

target_include_directories(myground_leishen PRIVATE
	${PCL_INCLUDE_DIRS}
)

target_link_libraries(myground_leishen
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
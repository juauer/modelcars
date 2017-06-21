cmake_minimum_required( VERSION 2.8.3 )
set( CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}" )
project( map )

find_package( catkin REQUIRED COMPONENTS
  roscpp rosbag cv_bridge image_transport roslaunch
)

#roslaunch_add_file_check(launch)

find_package( OpenCV REQUIRED )

catkin_package(
	LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS cv_bridge image_transport message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}
)

add_executable( create_map src/create_map.cpp )
target_link_libraries( create_map ${catkin_LIBRARIES} ${OpenCV_LIBS} )

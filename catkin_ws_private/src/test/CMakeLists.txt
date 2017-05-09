cmake_minimum_required( VERSION 2.8.3 )
set( CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}" )
project( test )

find_package( catkin REQUIRED COMPONENTS
  roscpp cv_bridge image_transport message_generation std_msgs sensor_msgs
)

find_package( OpenCV REQUIRED )

catkin_package(
	LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS cv_bridge image_transport message_runtime std_msgs sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}
)

add_executable( test_node src/test.cpp )
target_link_libraries( test_node ${catkin_LIBRARIES} ${OpenCV_LIBS} )

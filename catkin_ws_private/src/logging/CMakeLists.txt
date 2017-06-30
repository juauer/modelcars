cmake_minimum_required( VERSION 2.8.3 )
set( CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}" )
project( logging )

find_package( catkin REQUIRED COMPONENTS
  roscpp rosbag cv_bridge image_transport roslaunch
)

roslaunch_add_file_check(launch)

find_package( OpenCV 2.4 REQUIRED )

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS cv_bridge image_transport message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}
)

add_executable( log_generator src/log_generator.cpp )
target_link_libraries( log_generator ${catkin_LIBRARIES} ${OpenCV_LIBS} )

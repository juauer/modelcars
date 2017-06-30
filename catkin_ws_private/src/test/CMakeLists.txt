cmake_minimum_required( VERSION 2.8.3 )
set( CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}" )
project( test )

find_package( catkin REQUIRED COMPONENTS roslib roscpp )
find_package( OpenCV 2.4 REQUIRED )

include_directories(
  ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
)

add_executable( trajectory_plotter src/trajectory_plotter.cpp )
target_link_libraries( trajectory_plotter ${catkin_LIBRARIES} ${OpenCV_LIBS} )

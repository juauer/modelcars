cmake_minimum_required( VERSION 2.8.3 )
set( CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}" )
project( test )

find_package( catkin REQUIRED COMPONENTS )

catkin_package(
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

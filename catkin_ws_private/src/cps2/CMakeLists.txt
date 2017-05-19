cmake_minimum_required( VERSION 2.8.3 )
set( CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}" )
project( cps2 )

find_package( catkin REQUIRED COMPONENTS
  roscpp image_transport
)

find_package( OpenCV REQUIRED )

include_directories(
  ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS image_transport
)

add_executable( test_evaluator src/image_evaluator.cpp src/map.cpp )
target_link_libraries( test_evaluator ${catkin_LIBRARIES} ${OpenCV_LIBS} )

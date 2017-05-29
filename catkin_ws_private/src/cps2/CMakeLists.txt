cmake_minimum_required( VERSION 2.8.3 )
set( CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}" )
project( cps2 )

find_package( catkin REQUIRED COMPONENTS
  roslaunch roscpp image_transport
)

roslaunch_add_file_check(launch)

find_package( OpenCV REQUIRED )

include_directories(
  ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS image_transport
)

add_executable( localization_publisher src/localization_publisher.cpp src/particle_filter.hpp src/particle3f.cpp src/image_evaluator.cpp src/particle3f.cpp src/map.cpp )
target_link_libraries( localization_publisher ${catkin_LIBRARIES} ${OpenCV_LIBS} )

add_executable( test_evaluator src/image_evaluator.cpp src/test_image_evaluator.cpp src/map.cpp)
target_compile_definitions( test_evaluator PUBLIC DEBUG_IE )
target_link_libraries( test_evaluator ${catkin_LIBRARIES} ${OpenCV_LIBS} )

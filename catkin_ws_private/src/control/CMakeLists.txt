cmake_minimum_required( VERSION 2.8.3 )
set( CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}" )
project( control )

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package( catkin REQUIRED COMPONENTS
  roscpp std_msgs sensor_msgs geometry_msgs
)

find_package( OpenCV REQUIRED )

include_directories(
  ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS std_msgs sensor_msgs geometry_msgs
)

add_executable( control_node src/control_node.cpp )
target_link_libraries( control_node ${catkin_LIBRARIES} )

add_executable( control_wasd src/control_wasd.cpp )
target_link_libraries( control_wasd ${catkin_LIBRARIES} )

install( TARGETS control_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install( DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

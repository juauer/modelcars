cmake_minimum_required( VERSION 2.8.3 )
set( CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}" )
project( control )

find_package( catkin REQUIRED COMPONENTS
  roslib roscpp std_msgs sensor_msgs geometry_msgs
)

find_package( OpenCV REQUIRED )

include_directories(
  ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS std_msgs sensor_msgs geometry_msgs
)

add_executable( control_wasd src/control_wasd.cpp )
target_link_libraries( control_wasd ${catkin_LIBRARIES} )

add_executable(control_node src/control_node.cpp)
target_link_libraries(control_node ${catkin_LIBRARIES})

add_executable(path_control_node src/path_control_node.cpp)
target_link_libraries(path_control_node ${catkin_LIBRARIES})

add_executable(control_node_debug src/control_node.cpp)
target_compile_definitions( control_node_debug PUBLIC DEBUG_CONTROL )
target_link_libraries(control_node_debug ${catkin_LIBRARIES})

add_executable(path_control_node_debug src/path_control_node.cpp)
target_compile_definitions( path_control_node_debug PUBLIC DEBUG_CONTROL )
target_link_libraries(path_control_node_debug ${catkin_LIBRARIES})

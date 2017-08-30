cmake_minimum_required( VERSION 2.8.3 )
add_compile_options(-std=c++11)
project( control )

find_package( catkin REQUIRED COMPONENTS
  roslib roscpp std_msgs sensor_msgs geometry_msgs cps2_particle_msgs
)

find_package( OpenCV REQUIRED )

include_directories(
  ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS std_msgs sensor_msgs geometry_msgs cps2_particle_msgs
)

add_executable( control_wasd src/control_wasd.cpp )
target_link_libraries( control_wasd ${catkin_LIBRARIES} )

add_executable( control_node src/control_node.cpp )
target_link_libraries(control_node ${catkin_LIBRARIES})

add_executable( control_track_node src/control_track_node.cpp )
target_link_libraries( control_track_node ${catkin_LIBRARIES} )

add_executable( control_node_debug src/control_node.cpp )
target_compile_definitions( control_node_debug PUBLIC DEBUG_CONTROL )
target_link_libraries( control_node_debug ${catkin_LIBRARIES} )

add_executable( control_track_node_debug src/control_track_node.cpp )
target_compile_definitions( control_track_node_debug PUBLIC DEBUG_CONTROL )
target_link_libraries( control_track_node_debug ${catkin_LIBRARIES} )


install( TARGETS control_wasd control_node control_track_node control_node_debug control_track_node_debug
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install( DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

install( DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

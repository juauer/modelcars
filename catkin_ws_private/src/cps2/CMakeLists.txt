cmake_minimum_required( VERSION 2.8.3 )
set( CMAKE_CXX_FLAGS "-std=gnu++11 ${CMAKE_CXX_FLAGS}" )

project( cps2 )

find_package( catkin REQUIRED COMPONENTS
  roslaunch roscpp image_transport cv_bridge fisheye_camera_matrix
)

roslaunch_add_file_check( launch )

find_package( OpenCV REQUIRED )

include_directories(
  ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS cv_bridge image_transport fisheye_camera_matrix
)

add_executable( localization_publisher src/localization_publisher.cpp src/image_evaluator.cpp src/map.cpp src/particle_filter.cpp )
target_link_libraries( localization_publisher ${catkin_LIBRARIES} ${OpenCV_LIBS} )

add_executable( localization_publisher_debug src/localization_publisher.cpp src/image_evaluator.cpp src/map.cpp src/particle_filter.cpp )
target_compile_definitions( localization_publisher_debug PUBLIC DEBUG_PF )
target_link_libraries( localization_publisher_debug ${catkin_LIBRARIES} ${OpenCV_LIBS} )

add_executable( localization_publisher_debug_static src/localization_publisher.cpp src/image_evaluator.cpp src/map.cpp src/particle_filter.cpp )
target_compile_definitions( localization_publisher_debug_static PUBLIC DEBUG_PF DEBUG_PF_STATIC )
target_link_libraries( localization_publisher_debug_static ${catkin_LIBRARIES} ${OpenCV_LIBS} )

add_executable( test_evaluator src/image_evaluator.cpp src/test/test_image_evaluator.cpp src/map.cpp )
target_compile_definitions( test_evaluator PUBLIC DEBUG_IE )
target_link_libraries( test_evaluator ${catkin_LIBRARIES} ${OpenCV_LIBS} )

add_executable( test_image_distance src/test/test_image_distance.cpp src/map.cpp src/image_evaluator.cpp )
target_link_libraries( test_image_distance ${catkin_LIBRARIES} ${OpenCV_LIBS} )

add_executable( test_map_transforms src/test/test_map_transforms.cpp src/image_evaluator.cpp )
target_link_libraries( test_map_transforms ${catkin_LIBRARIES} ${OpenCV_LIBS} )

add_executable( trajectory_plotter src/test/trajectory_plotter.cpp )
target_link_libraries( trajectory_plotter ${catkin_LIBRARIES} ${OpenCV_LIBS} )

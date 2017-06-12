cmake_minimum_required( VERSION 2.8.3 )
set( CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}" )
project( camera_matrix )

find_package( catkin REQUIRED COMPONENTS
  roscpp cv_bridge image_transport message_generation camera_matrix_msgs
)

find_package( OpenCV REQUIRED )

include_directories(
  ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS cv_bridge image_transport message_runtime camera_matrix_msgs
)

add_executable( offline_calibration_manual src/offline_calibration_manual.cpp src/camera_matrix.cpp )
target_link_libraries( offline_calibration_manual ${catkin_LIBRARIES} ${OpenCV_LIBS} )

add_executable( camera_matrix_publisher src/camera_matrix_publisher.cpp src/camera_matrix.cpp )
target_link_libraries( camera_matrix_publisher ${catkin_LIBRARIES} ${OpenCV_LIBS} )

add_executable( inverse_perspective_publisher src/inverse_perspective_tf_publisher.cpp src/camera_matrix.cpp )
target_link_libraries( inverse_perspective_publisher ${catkin_LIBRARIES} ${OpenCV_LIBS} )

add_executable( undistorted_image_publisher src/undistorted_image_publisher.cpp src/camera_matrix.cpp )
target_link_libraries( undistorted_image_publisher ${catkin_LIBRARIES} ${OpenCV_LIBS} )

add_executable( test1 src/offline_calibration_ortho.cpp )
target_link_libraries( test1 ${catkin_LIBRARIES} ${OpenCV_LIBS} )

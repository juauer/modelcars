cmake_minimum_required( VERSION 2.8.3 )
set( CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}" )
project( camera_matrix )

find_package( catkin REQUIRED COMPONENTS
  roscpp cv_bridge image_transport std_msgs message_generation camera_matrix_msgs
)

find_package( OpenCV REQUIRED )

include_directories(
  ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
)

add_executable( calibration src/offline_calibration.cpp src/camera_matrix.cpp )
target_link_libraries( calibration ${catkin_LIBRARIES} ${OpenCV_LIBS} )

add_executable( cm_pub src/camera_matrix_publisher.cpp src/camera_matrix.cpp )
target_link_libraries( cm_pub ${catkin_LIBRARIES} ${OpenCV_LIBS} )

add_executable( ip_pub src/inverse_perspective_tf_publisher.cpp src/camera_matrix.cpp )
target_link_libraries( ip_pub ${catkin_LIBRARIES} ${OpenCV_LIBS} )

add_executable( ud_pub src/undistorted_image_publisher.cpp src/camera_matrix.cpp )
target_link_libraries( ud_pub ${catkin_LIBRARIES} ${OpenCV_LIBS} )

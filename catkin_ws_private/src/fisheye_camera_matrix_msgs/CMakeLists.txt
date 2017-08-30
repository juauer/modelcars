cmake_minimum_required( VERSION 2.8.3 )
add_compile_options(-std=c++11)
project( fisheye_camera_matrix_msgs )

find_package( catkin REQUIRED COMPONENTS
  message_generation std_msgs
)

add_message_files( FILES
  CameraMatrix.msg
)

generate_messages( DEPENDENCIES std_msgs )

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package()

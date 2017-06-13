#include <ros/ros.h>
#include <ros/package.h>

#include "camera_matrix.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "camera_matrix_publisher");

  if(argc < 2) {
    ROS_ERROR("Please use roslaunch: 'roslaunch fisheye_camera_matrix camera_matrix_publisher.launch "
              "[calib:=FILE]'");
    return 1;
  }

  std::string path = ros::package::getPath("fisheye_camera_matrix") + std::string("/config/") + std::string(argv[1]);

  if(access(path.c_str(), R_OK ) == -1) {
    ROS_ERROR("No such file: %s\nPlease give a path relative to catkin_ws/src/camera_matrix/config/", path.c_str() );
    return 1;
  }

  ROS_INFO("camera_matrix_publisher: using calibrationfile: %s", path.c_str() );

  bool auto_calibration_enabled = false;

  // TODO add switch to enable auto calibration

  fisheye_camera_matrix::CameraMatrix camera_matrix(path.c_str() );
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<fisheye_camera_matrix_msgs::CameraMatrix>("/usb_cam/camera_matrix", 1);

  while(ros::ok() ) {
    ros::spinOnce();

    if(auto_calibration_enabled)
      camera_matrix.update_calibration();

    pub.publish(camera_matrix.serialize() );
  }
}

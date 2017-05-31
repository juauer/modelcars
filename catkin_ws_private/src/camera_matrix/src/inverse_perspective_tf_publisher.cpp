#include <ros/ros.h>

#include "camera_matrix.hpp"

void callback_camera_matrix(const camera_matrix_msgs::CameraMatrix &msg) {
  CameraMatrix camera_matrix(msg);

  // TODO use the camera matrix to compute and publish another matrix
  // for the inverse perspective transform (image coords -> world coords)
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "inverse_perspective_tf_publisher");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/usb_cam/camera_matrix", 1, &callback_camera_matrix);

  ros::spin();
}

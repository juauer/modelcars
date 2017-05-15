#include <ros/ros.h>

#include "camera_matrix.hpp"

int main(int argc, char **argv) {
	if(argc < 2) {
		ROS_ERROR("Missing argument: path to camera matrix config");
		return 1;
	}

	CameraMatrix camera_matrix(argv[1]);

	bool auto_calibration_enabled = false;

	// TODO add switch to enable auto calibration

	ros::init(argc, argv, "camera_matrix_publisher");
	ros::NodeHandle nh;

	ros::Publisher pub = nh.advertise<camera_matrix_msgs::CameraMatrix>("/usb_cam/camera_matrix", 1);

	while(ros::ok() ) {
		ros::spinOnce();

		if(auto_calibration_enabled)
			camera_matrix.calibrate_online_update();

		pub.publish(camera_matrix.serialize() );
	}
}

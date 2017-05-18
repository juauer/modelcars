#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "camera_matrix.hpp"

bool ready = false;

CameraMatrix camera_matrix;
image_transport::Publisher pub;
sensor_msgs::ImagePtr msg_undistorted;
cv::Mat img_raw, img_undistorted;

void callback_camera_matrix(const camera_matrix_msgs::CameraMatrix &msg) {
	camera_matrix = CameraMatrix(msg);
	ready         = true;
}

void callback_image(const sensor_msgs::ImageConstPtr &msg) {
	if(!ready)
		return;

	img_raw = cv_bridge::toCvShare(msg, "bgr8")->image;

	camera_matrix.undistort(img_raw, img_undistorted);

	msg_undistorted = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_undistorted).toImageMsg();

	msg_undistorted->header.seq = msg->header.seq;
	msg_undistorted->header.stamp = msg->header.stamp;

	pub.publish(msg_undistorted);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "undistorted_image_publisher");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);

	ros::Subscriber sub_cm              = nh.subscribe("/usb_cam/camera_matrix", 1, &callback_camera_matrix);
	image_transport::Subscriber sub_img = it.subscribe("/usb_cam/image_raw", 1, &callback_image);
	image_transport::Publisher pub      = it.advertise("/usb_cam/image_undistorted", 1);

	ros::spin();
}

#include <unistd.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Int64.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

char *path;
cv::Mat image;
image_transport::Publisher pub_image;
ros::Publisher pub_width, pub_height;
sensor_msgs::ImagePtr msg_image;
std_msgs::Int64 msg_width, msg_height;

void execute() {
	image           = cv::imread(path);
	msg_image       = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
	msg_width.data  = image.cols;
	msg_height.data = image.rows;

	pub_width.publish(msg_width);
	pub_height.publish(msg_height);
}

int main(int argc, char **argv) {
	if(argc < 2) {
		ROS_ERROR("Missing argument: image path");
		return 1;
	}

	path = argv[1];

	if(access(path, F_OK ) == -1) {
		ROS_ERROR("No such file: %s", path);
		return 1;
	}

	ros::init(argc, argv, "test_node");

	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);

	pub_image  = it.advertise("test/image", 1);
	pub_width  = nh.advertise<std_msgs::Int64>("test/width", 1);
	pub_height = nh.advertise<std_msgs::Int64>("test/height", 1);

	ROS_INFO("now publishing %s", path);

	while (ros::ok() ) {
		execute();
        ros::spinOnce();
	}

	return 0;
}

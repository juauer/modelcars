#include <string.h>
#include <vector>
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <rosbag/bag.h>
#include <tf/tf.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>

geometry_msgs::PoseStamped createPoseMsg(float x, float y, float th) {
	geometry_msgs::PoseStamped msg;

	msg.header.frame_id  = "/base_link";
	msg.pose.position.x  = x;
	msg.pose.position.y  = y;
	msg.pose.position.z  = 0.0;
	msg.pose.orientation = tf::createQuaternionMsgFromYaw(th);

	return msg;
}

nav_msgs::Odometry createOdometryMsg(float x, float y, float th, float vx, float vy, float vth) {
	nav_msgs::Odometry msg;

	msg.header.frame_id       = "/odom";
	msg.pose.pose.position.x  = x;
	msg.pose.pose.position.y  = y;
	msg.pose.pose.position.z  = 0.0;
	msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(th);
	msg.twist.twist.linear.x  = vx;
	msg.twist.twist.linear.y  = vy;
	msg.twist.twist.angular.z = vth;

	return msg;
}

sensor_msgs::ImagePtr createImgMsg(std::string path) {
	return cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv::imread(path) ).toImageMsg();
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "log_generator");

	ros::NodeHandle nh;
	rosbag::Bag bag;
	std::vector<geometry_msgs::PoseStamped> poses;
	std::vector<nav_msgs::Odometry> odos;
	std::vector<sensor_msgs::ImagePtr> images;
	nav_msgs::Path path;

	path.header.frame_id = "/base_link";
	std::string path_img = ros::package::getPath("logging") + std::string("/../../../captures/");
	std::string path_bag = ros::package::getPath("logging") + std::string("/../../../logs/test.bag");

	bag.open(path_bag, rosbag::bagmode::Write);

	poses.push_back(createPoseMsg(0, 0, 0) );
	poses.push_back(createPoseMsg(2, 0, 0) );
	poses.push_back(createPoseMsg(4, 0, 0) );
	poses.push_back(createPoseMsg(4, 2, 0) );
	poses.push_back(createPoseMsg(2, 2, 0) );
	poses.push_back(createPoseMsg(0, 2, 0) );

	odos.push_back(createOdometryMsg(0, 0, 0,  0, -2, 0) );
	odos.push_back(createOdometryMsg(2, 0, 0,  2,  0, 0) );
	odos.push_back(createOdometryMsg(4, 0, 0,  2,  0, 0) );
	odos.push_back(createOdometryMsg(4, 2, 0,  0,  2, 0) );
	odos.push_back(createOdometryMsg(2, 2, 0, -2,  0, 0) );
	odos.push_back(createOdometryMsg(0, 2, 0, -2,  0, 0) );

	images.push_back(createImgMsg(path_img + std::string("(0,0,0).jpg") ) );
	images.push_back(createImgMsg(path_img + std::string("(200,0,0).jpg") ) );
	images.push_back(createImgMsg(path_img + std::string("(400,0,0).jpg") ) );
	images.push_back(createImgMsg(path_img + std::string("(400,200,0).jpg") ) );
	images.push_back(createImgMsg(path_img + std::string("(200,200,0).jpg") ) );
	images.push_back(createImgMsg(path_img + std::string("(0,200,0).jpg") ) );

	ROS_INFO("generating ...");

	int i       = 0;
	int j       = 0;
	int s       = images.size();
	ros::Time t = ros::Time::now();

	while (ros::ok() && i < 64) {
		i     += 1;
		t.sec += 1;
		j      = i % s;

		poses[j].header.seq     = i;
		poses[j].header.stamp   = t;
		odos[j].header.seq      = i;
		odos[j].header.stamp    = t;
		images[j]->header.seq   = i;
		images[j]->header.stamp = t;
		path.header.seq     = i;
		path.header.stamp   = t;

		if(i > 3)
			path.poses.erase(path.poses.begin() );

		path.poses.push_back(poses[j]);
		bag.write("/path", t, path);
		bag.write("/odom", t, odos[j]);
		bag.write("/usb_cam/image_raw", t, images[j]);
	}

	bag.close();
	ROS_INFO("... done.");
	return 0;
}

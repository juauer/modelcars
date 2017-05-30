#include <stdlib.h>
#include <vector>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/core.hpp>
#include <ros/ros.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "particle_filter_dummy.hpp"
// FIXME #include "particle_filter.hpp"

bool ready = false;

cps2::Map *map;
dummy::ParticleFilter *particleFilter;
// FIXME cps2::ParticleFilter3f *particleFilter;

cv::Mat image;
dummy::Particle pose;
// FIXME cps2::Particle3f pose;
geometry_msgs::PoseStamped msg_pose;
nav_msgs::Odometry msg_odo;
ros::Publisher pub;

#ifdef DEBUG_PF
visualization_msgs::Marker markers[particleFilter.particleCount];
visualization_msgs::MarkerArray msg_markers;
ros::Publisher pub_markers;
#endif

void callback_odometry(const nav_msgs::Odometry &msg) {
  msg_odo = msg;
  ready = true;
}

void callback_image(const sensor_msgs::ImageConstPtr &msg) {
  if (!ready)
    return;

  image = cv_bridge::toCvShare(msg, "bgr8")->image;

  // TODO transform odometry velocities (world coords) to (dx, dy) (image coords)
  particleFilter->evaluate(image, 0, 0);

  dummy::Particle pose = particleFilter->getBest();
  // FIXME cps2::Particle3f pose = particleFilter->getBest();

  msg_pose.header.seq = msg->header.seq;
  msg_pose.header.stamp = msg->header.stamp;
  msg_pose.pose.position.x = pose.p.x;
  msg_pose.pose.position.y = pose.p.y;
  msg_pose.pose.orientation = tf::createQuaternionFromYaw(pose.p.z);
  pub.publish(msg_pose);

#ifdef DEBUG_PF
  for(int i = 0; i < particleFilter.particleCount; ++i) {
	  tf::Quaternion q = tf::createQuaternionFromYaw(pose.p.z);
	  visualization_msgs::Marker marker = markers[i];
	  marker.id = i;
	  marker.pose.position.x = pose.p.x;
	  marker.pose.position.y = pose.p.y;
	  marker.pose.orientation.x = q.getX();
	  marker.pose.orientation.y = q.getY();
	  marker.pose.orientation.z = q.getZ();
	  marker.pose.orientation.w = q.getW();
  }

  pub_markers.publish(msg_markers);
#endif
}

int main(int argc, char **argv) {
	if(argc < 4) {
		ROS_ERROR("Please use roslaunch as entry point.");
		return 1;
	}

	std::string path_map = ros::package::getPath("cps2") + std::string("/../../../captures/") + std::string(argv[1]);
	int particles_num    = atoi(argv[2]);
	int particles_keep   = atof(argv[3]);
	int particle_stddev  = atof(argv[4]);

	if(access(path_map.c_str(), R_OK ) == -1) {
		ROS_ERROR("No such file: %s\nPlease give a path relative to catkin_ws/../captures/.\n", path_map.c_str() );
		return 1;
	}

	map = new cps2::Map(path_map.c_str());
	particleFilter = new dummy::ParticleFilter(map, particles_num, particles_keep, particle_stddev);
	// FIXME particleFilter = new cps2::ParticleFilter3f(map, particles_num, particles_keep, particle_stddev);

  ros::init(argc, argv, "localization_cps2_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  ros::Subscriber sub_odo = nh.subscribe("/odom", 1, &callback_odometry);
  image_transport::Subscriber sub_img =
      it.subscribe("/usb_cam/image_undistorted", 1, &callback_image);

  msg_pose.pose.position.z = 0;
  pub = nh.advertise<geometry_msgs::PoseStamped>("/localization/cps2/pose", 1);

#ifdef DEBUG_PF
  pub_markers = nh.advertise<visualization_msgs::MarkerArray>("/localization/cps2/particles", 1);
  msg_markers.markers = markers;

  for(int i = 0; i < particleFilter.particleCount; ++i) {
	  tf::Quaternion q = tf::createQuaternionFromYaw(pose.p.z);
	  visualization_msgs::Marker marker = markers[i];
	  marker.header.frame_id = "base_link";
	  marker.header.stamp = 0;
	  marker.ns = "cps2";
	  marker.type = visualization_msgs::Marker::ARROW;
	  marker.action = visualization_msgs::Marker::ADD;
	  marker.pose.position.z = 0;
	  marker.scale.x = 0.5;
	  marker.scale.y = 0.5;
	  marker.scale.z = 0.5;
	  marker.color.a = 1.0;
	  marker.color.r = 0.0;
	  marker.color.g = 1.0;
	  marker.color.b = 0.0;
  }
#endif

  ros::spin();
}

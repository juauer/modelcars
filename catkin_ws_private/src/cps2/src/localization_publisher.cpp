#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/core.hpp>
#include <ros/ros.h>
#include <tf/tf.h>
#include "particle3f.hpp"

bool ready = false;

cps2::Map map;
cps2::ParticleFilter3f pf(map,1000);

cv::Mat image;
cv::Point3f pose;
geometry_msgs::PoseStamped msg_pose;
nav_msgs::Odometry msg_odo;
ros::Publisher pub;

Particle3f convOdomMsg2Particle3f(nav_msgs::Odometry msg) {
  Particle3f p;
  p.p.x = msg.pose.position.x;
  p.p.y = msg.pose.position.y;
  p.p.z = msg.pose.position.z;

  //msg.twist.twist.linear.x  = vx;
  //msg.twist.twist.linear.y  = vy;
  //msg.twist.twist.angular.z = 0.0;
  
  return p;
}


void callback_odometry(const nav_msgs::Odometry &msg) {
  msg_odo = msg;
  ready = true;
}

void callback_image(const sensor_msgs::ImageConstPtr &msg) {
  if (!ready)
    return;

  image = cv_bridge::toCvShare(msg, "bgr8")->image;
  particleFilter.evaluate(image, convOdomMsg2Particle3f(msg_odo));
  Particle3f pose  = particleFilter.getBest();
  msg_pose.header.seq = msg->header.seq;
  msg_pose.header.stamp = msg->header.stamp;
  msg_pose.pose.position.x = pose.x;
  msg_pose.pose.position.y = pose.y;
  msg_pose.pose.orientation = tf::createQuaternionFromYaw(pose.z);
  pub.publish(msg_pose);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "localization_cps2_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  ros::Subscriber sub_odo = nh.subscribe("/odom", 1, &callback_odometry);
  image_transport::Subscriber sub_img =
      it.subscribe("/usb_cam/image_undistorted", 1, &callback_image);

  msg_pose.pose.position.z = 0;
  pub = nh.advertise<geometry_msgs::PoseStamped>("/localization/cps2/pose", 1);

  ros::spin();
}

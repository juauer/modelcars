#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/core.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/tf.h>
#include "particle3f.hpp"

bool ready = false;

cps2::Map *map;
cps2::ParticleFilter3f pf(*map,1000);

cv::Mat image;
cv::Point3f pose;
geometry_msgs::PoseStamped msg_pose;
nav_msgs::Odometry msg_odo;
ros::Publisher pub;

cps2::Particle3f convOdomMsg2Particle3f(nav_msgs::Odometry msg) {
  cps2::Particle3f p(map);
  p.p.x = msg.pose.pose.position.x;
  p.p.y = msg.pose.pose.position.y;
  p.p.z = msg.pose.pose.position.z;

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
  pf.eval(image, convOdomMsg2Particle3f(msg_odo));
  cps2::Particle3f pose = pf.getBest();
  msg_pose.header.seq = msg->header.seq;
  msg_pose.header.stamp = msg->header.stamp;
  msg_pose.pose.position.x = pose.p.x;
  msg_pose.pose.position.y = pose.p.y;
  //msg_pose.pose.orientation = tf::createQuaternionFromYaw(pose.p.z);
  pub.publish(msg_pose);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "localization_cps2_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  std::string path = ros::package::getPath("cps2") +
                     std::string("/../../../captures/map_test.jpg");

  map = new cps2::Map(path.c_str());
  
  ros::Subscriber sub_odo = nh.subscribe("/odom", 1, &callback_odometry);
  image_transport::Subscriber sub_img =
      it.subscribe("/usb_cam/image_undistorted", 1, &callback_image);

  msg_pose.pose.position.z = 0;
  pub = nh.advertise<geometry_msgs::PoseStamped>("/localization/cps2/pose", 1);

  ros::spin();
}

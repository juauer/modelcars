#include <math.h>
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

cv::Mat map;
std::vector<geometry_msgs::PoseStamped> poses;
std::vector<nav_msgs::Odometry> odos;
std::vector<sensor_msgs::ImagePtr> images;

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

sensor_msgs::ImagePtr createImgMsg(float x, float y, float th) {
  x  *= 100;
  y   = map.rows - 100 * y;
  th -= M_PI / 2;

  int dim_y = 240;
  int dim_x = 300;
  int cx    = dim_x / 2;
  int cy    = dim_y / 2;

  cv::Mat mappiece(dim_y, dim_x, CV_8UC3);

  for(int r = 0; r < dim_y; ++r)
    for(int c = 0; c < dim_x; ++c) {
      int sx   = c - cx;
      int sy   = r - cy;
      float xx = sx * cos(th) - sy * sin(th) + x;
      float yy = sx * sin(th) + sy * cos(th) + y;

      if(xx >= 0 && yy >= 0 && xx < map.cols && yy < map.rows)
        mappiece.at<cv::Vec3b>(r, c) = map.at<cv::Vec3b>((int)yy, (int)xx);
      else
        mappiece.at<cv::Vec3b>(r, c) = cv::Vec3b(0, 0, 0);
    }

  return cv_bridge::CvImage(std_msgs::Header(), "bgr8", mappiece ).toImageMsg();
}

void createDataPoint(float x1, float x2, float y1, float y2, float th1, float th2) {
  poses.push_back(createPoseMsg(x1, y1, th1) );
  odos.push_back(createOdometryMsg(x1, y1, th1, 5 * (x1 - x2), 5 * (y1 - y2), 5 * (th1 - th2) ) );
  images.push_back(createImgMsg(x1, y1, th1) );
}

int main(int argc, char **argv) {
  if(argc < 2) {
    ROS_ERROR("Please use roslaunch as entry point.");
    return 1;
  }

  std::string path_map = ros::package::getPath("logging") + std::string("/../../../captures/") + std::string(argv[1]);

  if(access(path_map.c_str(), F_OK ) == -1) {
    ROS_ERROR("No such file: %s\nPlease give a path relative to catkin_ws/../captures/.\n", path_map.c_str() );
    return 1;
  }

  ros::init(argc, argv, "log_generator");
  ros::NodeHandle nh;
  rosbag::Bag bag;
  nav_msgs::Path path;

  map                  = cv::imread(path_map);
  path.header.frame_id = "/base_link";
  std::string path_img = ros::package::getPath("logging") + std::string("/../../../captures/");
  std::string path_bag = ros::package::getPath("logging") + std::string("/../../../logs/test.bag");

  bag.open(path_bag, rosbag::bagmode::Write);
  ROS_INFO("generating ...");

  float x1  = 3;
  float x2  = 3;
  float y1  = 2;
  float y2  = 2;
  float th1 = 0;
  float th2 = 0;

  for(int i = 0; i < 8; ++i) {
    for(int j = 0; j < 20; ++j) {
      createDataPoint(x1, x2, y1, y2, th1, th2);
      x2  = x1;
      y2  = y1;
      th2 = th1;
      x1 += 0.1 * cos(th1);
      y1 += 0.1 * sin(th1);
    }

    for(int j = 0; j < 20; ++j) {
      createDataPoint(x1, x2, y1, y2, th1, th2);
      x2   = x1;
      y2   = y1;
      th2  = th1;
      x1  += 0.1 * cos(th1);
      y1  += 0.1 * sin(th1);
      th1 += M_PI / 40;
    }
  }

  int s       = images.size();
  ros::Time t = ros::Time::now();

  for(int i = 0; i < s; ++i) {
    t.nsec += 200000000;

    if(t.nsec >= 1000000000) {
      ++t.sec;
      t.nsec = 0;
    }

    poses[i].header.seq     = i;
    poses[i].header.stamp   = t;
    odos[i].header.seq      = i;
    odos[i].header.stamp    = t;
    images[i]->header.seq   = i;
    images[i]->header.stamp = t;
    path.header.seq         = i;
    path.header.stamp       = t;

    if(i > 15)
      path.poses.erase(path.poses.begin() );

    path.poses.push_back(poses[i]);
    bag.write("/path", t, path);
    bag.write("/odom", t, odos[i]);
    bag.write("/usb_cam/image_raw", t, images[i]);
  }

  bag.close();
  ROS_INFO("... done.");
  return 0;
}

#include <ros/ros.h>
#include <tf/tf.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/PoseStamped.h>
#include <cps2_particle_msgs/particle_msgs.h>

std_msgs::Int16 msg_speed;
std_msgs::Int16 msg_steering;

void callback_speed(const std_msgs::Int16 &msg) {
  msg_speed = msg;
}

void callback_steering(const std_msgs::Int16 &msg) {
  msg_steering = msg;
}

/**
 * Publish a fake localization based on the published speed and steering.
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "test_control");
  ros::NodeHandle nh;
  geometry_msgs::PoseStamped msg_pose;
  cps2_particle_msgs::particle_msgs msg_particle;

  msg_pose.header.frame_id     = "base_link";
  msg_pose.pose.position.x     = 0.5;
  msg_pose.pose.position.y     = 0.5;
  msg_pose.pose.position.z     = 0;
  msg_pose.pose.orientation.w  = 1;
  msg_pose.pose.orientation.x  = 0;
  msg_pose.pose.orientation.y  = 0;
  msg_pose.pose.orientation.z  = 0;
  msg_particle.header.frame_id = "base_link";
  msg_particle.belief          = 1;
  msg_speed.data               = 0;
  msg_steering.data            = 90;

  ros::Subscriber sub_speed    = nh.subscribe("/manual_control/speed", 1, &callback_speed);
  ros::Subscriber sub_steering = nh.subscribe("/manual_control/steering", 1, &callback_steering);
  ros::Publisher pub_pose      = nh.advertise<geometry_msgs::PoseStamped>(
       "/localization/cps2/pose", 1);
  ros::Publisher pub_particle  = nh.advertise<cps2_particle_msgs::particle_msgs>(
      "/localization/cps2/particle", 1);
  ros::Rate r                  = 10;

  while(ros::ok() ) {
    r.sleep();
    ros::spinOnce();

    tf::Quaternion q_last;

    tf::quaternionMsgToTF(msg_pose.pose.orientation, q_last);

    float speed    = -msg_speed.data;
    float steering = /* '-' ?*/ (msg_steering.data - 90) * M_PI / 180;
    float yaw_last = tf::getYaw(q_last);
    float yaw_new  = yaw_last + 0.001 * speed * steering;

    msg_pose.pose.position.x += 0.0001 * speed * cosf(yaw_new);
    msg_pose.pose.position.y += 0.0001 * speed * sinf(yaw_new);
    tf::Quaternion q_new      = tf::createQuaternionFromYaw(yaw_new);

    tf::quaternionTFToMsg(q_new, msg_pose.pose.orientation);

    msg_particle.pose = msg_pose.pose;

    pub_pose.publish(msg_pose);
    pub_particle.publish(msg_particle);

    // TODO check sign of steering (see above)
    // ROS_INFO("x: %.2f  y: %.2f  yaw: %.2f", msg_particle.pose.position.x, msg_particle.pose.position.y, tf::getYaw(q_new) );
  }

  return 0;
}

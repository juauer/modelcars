#include "ros/ros.h"
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>

//parameters
#include <ros/ros.h>

class Control {
 public:
  Control(ros::NodeHandle nh) {
    n_.param<int>("mode", mode, 0);
    pubSpeed_=nh.advertise<std_msgs::Int16>(nh.resolveName("manual_control/speed"), 1);
    pubSteering_=nh.advertise<std_msgs::Int16>(nh.resolveName("manual_control/steering"), 1);
    //subScan_ = n_.subscribe("scan", 1, &Control::scanCallback,this);
    subTwist_ = n_.subscribe("motor_control/twist",1,&Control::speedCallback,this); 
  }
  ~Control(){}

  void speedCallback(const geometry_msgs::Twist& twist) {
    direction=twist.linear.x;
  }

  // void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
  //   int count = scan->scan_time / scan->time_increment;
  //   float  break_distance_=break_distance;
  //   ros::Time begin = ros::Time::now();
  //   if (abs(direction)>500)
  //     break_distance_=(abs(direction)/500)*break_distance;
  //   //ROS_INFO("speed %f",break_distance_);	
  //   if(direction < 0){	//backw.
  //     for(int i = 0; i < (angle_back/2)+1; i++) {
  //       if (scan->ranges[i] <= break_distance_) {
  //         speed.data=5000;
  //         pubSpeed_.publish(speed);
  //         steering_angle.data=90-steering_angle_data;
  //         speed.data=0;
  //         pubSpeed_.publish(speed);
  //         pubSteering_.publish(steering_angle);
  //         ros::Duration(0.5).sleep();
  //         speed.data=1000;
  //         pubSpeed_.publish(speed);
  //         //ROS_INFO("Obstacle");
  //         return;
  //       }
  //     }
  //     for(int k = (360-(angle_back/2)); k < count; k++) {
  //       if (scan->ranges[k] <= break_distance_) {
  //         speed.data=5000;
  //         pubSpeed_.publish(speed);
  //         steering_angle.data=90-steering_angle_data;
  //         speed.data=0;
  //         pubSpeed_.publish(speed);
  //         pubSteering_.publish(steering_angle);
  //         ros::Duration(0.5).sleep();
  //         speed.data=1000;
  //         pubSpeed_.publish(speed);
  //         return;
  //       }
  //     }
  //   }

  //   if(direction > 0) { //forw.
  //     for(int j = (180-(angle_front/2)); j < (180+(angle_front/2))+1; j++) {
  //       if (scan->ranges[j] <= break_distance_) {
  //         speed.data=-5000;
  //         pubSpeed_.publish(speed);
  //         steering_angle.data=90+steering_angle_data;
  //         speed.data=0;
  //         pubSpeed_.publish(speed);
  //         pubSteering_.publish(steering_angle);
  //         ros::Duration(0.5).sleep();
  //         speed.data=-1000;
  //         pubSpeed_.publish(speed);
  //         return;
  //       }
  //     }
  //   }
  // }

 private:
  std_msgs::Int16 steering_angle;
  std_msgs::Int16 speed;
  int direction;
  int mode;
  ros::NodeHandle n_; 
  ros::Publisher pubSpeed_;
  ros::Publisher pubSteering_;
  ros::Subscriber subTwist_;

};//End of class auto_stop

int main(int argc, char **argv) {
  ros::init(argc, argv, "control_node");
  ros::NodeHandle nh; 
  Control control(nh);

  while(ros::ok()) {
    ros::spin();
  }
  return 0;
}

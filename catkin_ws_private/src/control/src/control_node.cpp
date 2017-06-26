#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <opencv2/core.hpp>
#include <math.h> 

class Control {
 public:
  Control(ros::NodeHandle nh):seqNum(0) {
    n_.param<int>("mode", mode, 0);
    n_.param<int>("dstPosX", dstPosX, 0);
    n_.param<int>("dstPosY", dstPosY, 0);
    n_.param<int>("speed", speed, 20);

    dst.x = dstPosX;
    dst.y = dstPosY;
    dst.z = 0; // because we move on a 2d plane
    
    pubSteering_ = nh.advertise<std_msgs::Int16>(nh.resolveName("manual_control/steering"), 1);
    pubDst_ = nh.advertise<geometry_msgs::Point>(nh.resolveName("/control/destination"), 1);
    subDir_ = n_.subscribe("/localization/cps2/pose",1,&Control::setDirection,this); 
    subDst_ = n_.subscribe("/localization/cps2/dst",1,&Control::setDestination,this); 
  }
  ~Control(){}

  void setDirection(const geometry_msgs::PoseStamped& msg_pose) {
    if (mode ==0){
      seqNum = msg_pose.header.seq;

      cv::Point3f pos;
    
      pos.x = msg_pose.pose.position.x;
      pos.x = msg_pose.pose.position.y;

      cv::Point3f dir = dst - pos;
      float steering = atan2f(dir.y, dir.x);

      msg_pose.pose.orientation.x;
      msg_pose.pose.orientation.y;
      msg_pose.pose.orientation.z;
      msg_pose.pose.orientation.w;
      //TODO::get current orientation and correct steering

      steering_angle_msg.data = steering;
    
      //float distance = cv::norm(dst, pos);
      //speed_msg.data = distance;
      //pubSpeed_.publish(speed_msg); // set speed according to distance ?
      pubSteering_.publish(steering_angle_msg);
    }else{
      // just set a const angle
      steering_angle_msg.data = mode-91;
      pubSteering_.publish(steering_angle_msg);
    }
  }
  
  void setDestination(const geometry_msgs::Point& msg_dst) {
    dst.x = msg_dst.x;
    dst.y = msg_dst.y;
  }

  int seqNum;
  int mode;
  int dstPosX;
  int dstPosY;
  int speed;
 protected:
  cv::Point3f dst;
  std_msgs::Int16 speed_msg;
  std_msgs::Int16 steering_angle_msg;
  ros::NodeHandle n_; 
  ros::Publisher pubSteering_;
  ros::Publisher pubDst_;
  ros::Subscriber subDir_;
  ros::Subscriber subDst_;

};//End of class auto_stop

int main(int argc, char **argv) {
  ros::init(argc, argv, "control_node");
  ros::NodeHandle nh; 
  Control control(nh);
  
  ROS_INFO("Control_node: mode: %d desired:(%d,%d) speed: %d",
           control.mode, control.dstPosX, control.dstPosY, control.speed);
  
  while(ros::ok()) {
    ros::spin();
  }
  return 0;
}

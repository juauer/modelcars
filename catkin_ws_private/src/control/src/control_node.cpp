#include <ros/ros.h>
#include <tf/tf.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <opencv2/core/core.hpp>
#include <math.h> 

class Control {
 public:
  Control(ros::NodeHandle nh):seqNum(0), reached(false) {
    n_.param<int>("mode", mode, 0);
    n_.param<float>("epsilon", epsilon, 0.1);
    n_.param<float>("dstPosX", dstPosX, 0);
    n_.param<float>("dstPosY", dstPosY, 0);
    n_.param<float>("speed", speed, 20);

    dst.x = dstPosX;
    dst.y = dstPosY;
    dst.z = 0; // because we move on a 2d plane

    steeringPose_msg.header.frame_id = "base_link";
    steeringPose_msg.type = visualization_msgs::Marker::ARROW;
    steeringPose_msg.action = visualization_msgs::Marker::ADD;
    steeringPose_msg.pose.position.z = 0;
    steeringPose_msg.scale.y = 0.05; // width
    steeringPose_msg.scale.z = 0.05; // height
    steeringPose_msg.color.a = 1.0;
    steeringPose_msg.color.r = 0.0;
    steeringPose_msg.color.g = 0.0;
    steeringPose_msg.color.b = 1.0;

    dstPoint_msg.header.frame_id = "base_link";
    dstPoint_msg.point.z = 0;
    
    pubSteering_ = nh.advertise<std_msgs::Int16>(nh.resolveName("manual_control/steering"), 1);
    pubDstReached_ = nh.advertise<std_msgs::Bool>(nh.resolveName("/localization/control/destination/reached"), 1);
    subDir_ = n_.subscribe("/localization/cps2/pose",1,&Control::setDirection,this); 
    subDst_ = n_.subscribe("/localization/cps2/dst",1,&Control::setDestination,this);
#ifdef DEBUG_CONTROL
    pubSteeringPose_ = nh.advertise<visualization_msgs::Marker>(nh.resolveName("/localization/control/SteeringPose"), 1);
    pubDst_ = nh.advertise<geometry_msgs::PointStamped>(nh.resolveName("/localization/control/destination"), 1);    
#endif
  }
  ~Control(){}

  void setDirection(const geometry_msgs::PoseStamped& msg_pose) {
    if (mode ==0){
      // check for old message
      if (seqNum < msg_pose.header.seq){        
        seqNum = msg_pose.header.seq;
        reached = false;

        // calculate direction
        cv::Point3f pos;
    
        pos.x = msg_pose.pose.position.x;
        pos.y = msg_pose.pose.position.y;

        cv::Point3f dir = dst - pos;
        float steering_rad = atan2f(dir.y, dir.x);
        float steering = std::min(std::max(steering_rad * (180.0/M_PI),-50.0),50.0);

        steering_angle_msg.data = steering;
    
        float distance = cv::sqrt(dir.x * dir.x + dir.y * dir.y);
        // speed_msg.data = distance;
        // pubSpeed_.publish(speed_msg); // set speed according to distance ?

        // check if destination is reached with epsilon precision
        // and send message that the destination is reached
        if( (dir.x < epsilon) && (-dir.x < epsilon) &&
            (dir.y < epsilon) && (-dir.y < epsilon)){
          ROS_INFO("control_node setDirection: destination reached");

          reached_msg.data = true;
          pubDstReached_.publish(reached_msg);
          // speed_msg.data = 0;
          // pubSpeed_.publish(speed_msg); // stop the car
        }

        pubSteering_.publish(steering_angle_msg);

#ifdef DEBUG_CONTROL
        dstPoint_msg.header.seq   = msg_pose.header.seq;
        dstPoint_msg.header.stamp = msg_pose.header.stamp;
        dstPoint_msg.point.x      = dst.x;
        dstPoint_msg.point.y      = dst.y;
        pubDst_.publish(dstPoint_msg);

        tf::Quaternion steering_q = tf::createQuaternionFromYaw(steering_rad);
        
        steeringPose_msg.header.seq         = msg_pose.header.seq;
        steeringPose_msg.header.stamp       = msg_pose.header.stamp;
        steeringPose_msg.pose.position.x    = msg_pose.pose.position.x;
        steeringPose_msg.pose.position.y    = msg_pose.pose.position.y;

        steeringPose_msg.pose.orientation.x = steering_q.getX();
        steeringPose_msg.pose.orientation.y = steering_q.getY();
        steeringPose_msg.pose.orientation.z = steering_q.getZ();
        steeringPose_msg.pose.orientation.w = steering_q.getW();

        steeringPose_msg.scale.x    = distance;//length
        
        pubSteeringPose_.publish(steeringPose_msg);
        ROS_INFO("control_node setDirection: pos(%.2f/%.2f) dst(%.2f/%.2f)",pos.x,pos.y, dst.x, dst.y);
#endif        
      }
    }else{
      // just send a const angle
      steering_angle_msg.data = mode-91;
      pubSteering_.publish(steering_angle_msg);
    }
  }
  
  void setDestination(const geometry_msgs::Point& msg_dst) {
    dstPosX = msg_dst.x;
    dstPosY = msg_dst.y;
    dst.x = msg_dst.x;
    dst.y = msg_dst.y;
    
#ifdef DEBUG_CONTROL
    ROS_INFO("control_node setDestination dst(%.2f/%.2f)", dst.x, dst.y);
#endif
  }

  int seqNum;
  int mode;
  float epsilon;
  float dstPosX;
  float dstPosY;
  float speed;
 protected:
  cv::Point3f dst;
  std_msgs::Int16 speed_msg;
  std_msgs::Int16 steering_angle_msg;
  std_msgs::Bool reached_msg;
  visualization_msgs::Marker steeringPose_msg;
  geometry_msgs::PointStamped dstPoint_msg;
  ros::NodeHandle n_; 
  ros::Publisher pubSteering_;
  ros::Publisher pubSteeringPose_;
  ros::Publisher pubDst_;
  ros::Subscriber subDir_;
  ros::Subscriber subDst_;

};//End of class auto_stop

int main(int argc, char **argv) {
  ros::init(argc, argv, "control_node");
  ros::NodeHandle nh; 
  Control control(nh);
  
  ROS_INFO("Control_node: mode: %d epsilon: %.2f desired:(%.2f,%.2f) speed: %.2f",
           control.mode, control.epsilon, control.dstPosX, control.dstPosY, control.speed);
  
  while(ros::ok()) {
    ros::spin();
  }
  return 0;
}

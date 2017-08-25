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

static const unsigned point_mode = 0;
static const unsigned angle_mode = 1;
// speed range [-1000...1000] direction is inverted
static const float max_speed = -200.0f;
static const float speed_multiplyer = -50.0f;
static const double max_steering_angle = 180.0f;
static const double min_steering_angle = 0.0f;

class Control {
 public:
  Control(ros::NodeHandle nh):seqNum(0), reached(false) {
    n_.param<int>("mode", mode, 0);
    n_.param<float>("epsilon", epsilon, 0.5);
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
    steeringPose_msg.color.r = 1.0;
    steeringPose_msg.color.g = 1.0;
    steeringPose_msg.color.b = 1.0;

    directionPose_msg.header.frame_id = "base_link";
    directionPose_msg.type = visualization_msgs::Marker::ARROW;
    directionPose_msg.action = visualization_msgs::Marker::ADD;
    directionPose_msg.pose.position.z = 0;
    directionPose_msg.scale.y = 0.05; // width
    directionPose_msg.scale.z = 0.05; // height
    directionPose_msg.color.a = 1.0;
    directionPose_msg.color.r = 0.0;
    directionPose_msg.color.g = 0.0;
    directionPose_msg.color.b = 1.0;
    
    dstPoint_msg.header.frame_id = "base_link";
    dstPoint_msg.point.z = 0;
    
    pubSteering_ = nh.advertise<std_msgs::Int16>(nh.resolveName("manual_control/steering"), 1);
    pubDstReached_ = nh.advertise<std_msgs::Bool>(nh.resolveName("/localization/control/destination/reached"), 1);
    subDir_ = n_.subscribe("/localization/cps2/pose",1,&Control::setDirection,this); 
    subDst_ = n_.subscribe("/localization/control/dest",1,&Control::setDestination,this);
    pubMotor_ = nh.advertise<std_msgs::Int16>("/manual_control/stop_start", 1);    
    pubSpeed_ = nh.advertise<std_msgs::Int16>("/manual_control/speed", 1);

    msg_stop.data     = 1;
    msg_start.data    = 0;
    // enable motors
    pubMotor_.publish(msg_start);

    if(mode==angle_mode){
      // set speed
      speed_msg.data = speed;
      pubSpeed_.publish(speed_msg);
    }

#ifdef DEBUG_CONTROL
    pubDirectionPose_ = nh.advertise<visualization_msgs::Marker>(nh.resolveName("/localization/control/DirectionPose"), 1);
    pubSteeringPose_ = nh.advertise<visualization_msgs::Marker>(nh.resolveName("/localization/control/SteeringPose"), 1);
    pubDst_ = nh.advertise<geometry_msgs::PointStamped>(nh.resolveName("/localization/control/destination"), 1);
#endif
  }
  ~Control(){}

  void setDirection(const geometry_msgs::PoseStamped& msg_pose) {
    // point mode
    if (mode == point_mode){
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
        //float steering = std::min(std::max(steering_rad * (180.0/M_PI),min_steering_angle),max_steering_angle);
        float steering = steering_rad * (180.0/M_PI);
        steering_angle_msg.data = steering;
    
        float distance = cv::sqrt(dir.x * dir.x + dir.y * dir.y);
        speed_msg.data = distance>epsilon ?std::max(max_speed,distance * speed_multiplyer):0;
        pubSpeed_.publish(speed_msg); // set speed according to distance ?

        // check if destination is reached with epsilon precision
        // and send message that the destination is reached
        if( (dir.x < epsilon) && (-dir.x < epsilon) &&
            (dir.y < epsilon) && (-dir.y < epsilon)){
#ifdef DEBUG_CONTROL
          ROS_INFO("control_node setDirection: destination reached");
#endif
          reached_msg.data = true;
          pubDstReached_.publish(reached_msg);
        }

        pubSteering_.publish(steering_angle_msg);

#ifdef DEBUG_CONTROL
        dstPoint_msg.header.seq   = msg_pose.header.seq;
        dstPoint_msg.header.stamp = msg_pose.header.stamp;
        dstPoint_msg.point.x      = dst.x;
        dstPoint_msg.point.y      = dst.y;
        pubDst_.publish(dstPoint_msg);

        tf::Quaternion direction_q = tf::createQuaternionFromYaw((steering/180.0f)*M_PI);
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

        directionPose_msg.header.seq         = msg_pose.header.seq;
        directionPose_msg.header.stamp       = msg_pose.header.stamp;
        directionPose_msg.pose.position.x    = msg_pose.pose.position.x;
        directionPose_msg.pose.position.y    = msg_pose.pose.position.y;

        directionPose_msg.pose.orientation.x = direction_q.getX();
        directionPose_msg.pose.orientation.y = direction_q.getY();
        directionPose_msg.pose.orientation.z = direction_q.getZ();
        directionPose_msg.pose.orientation.w = direction_q.getW();

        directionPose_msg.scale.x    = 1.0;//length
        
        pubSteeringPose_.publish(steeringPose_msg);
        pubDirectionPose_.publish(directionPose_msg);
        ROS_INFO("control_node setDirection: pos(%.2f/%.2f) dst(%.2f/%.2f) dist: %.2f speed: %d steering: %.2f",
                 pos.x,pos.y, dst.x, dst.y, distance, speed_msg.data, steering);
#endif
      }
      
    }else{ // angle_mode
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
  bool reached;
  std_msgs::Int16 speed_msg;
  std_msgs::Int16 steering_angle_msg;
  std_msgs::Bool reached_msg;
  visualization_msgs::Marker directionPose_msg;
  visualization_msgs::Marker steeringPose_msg;
  geometry_msgs::PointStamped dstPoint_msg;
  ros::NodeHandle n_; 
  ros::Publisher pubSteering_;
  ros::Publisher pubSteeringPose_;
  ros::Publisher pubDirectionPose_;
  ros::Publisher pubDst_;
  ros::Publisher pubDstReached_;
  ros::Publisher pubSpeed_;
  ros::Publisher pubMotor_;
  ros::Subscriber subDir_;
  ros::Subscriber subDst_;
  std_msgs::Int16 msg_stop;
  std_msgs::Int16 msg_start;

};//End of class auto_stop

int main(int argc, char **argv) {
  ros::init(argc, argv, "control_node");
  ros::NodeHandle nh; 
  Control control(nh);
#ifdef DEBUG_CONTROL
    ROS_INFO("control_node DEBUG_MODE");
#endif
  
  ROS_INFO("Control_node: mode: %d epsilon: %.2f desired:(%.2f,%.2f) speed: %.2f",
           control.mode, control.epsilon, control.dstPosX, control.dstPosY, control.speed);
  
  while(ros::ok()) {
    ros::spin();
  }
  return 0;
}

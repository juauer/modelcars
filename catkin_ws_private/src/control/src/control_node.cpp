#include <ros/ros.h>
#include <tf/tf.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <cps2_particle_msgs/particle_msgs.h>
#include <opencv2/core/core.hpp>
#include <math.h> 

static const unsigned point_mode = 0;
static const unsigned angle_mode = 1;
// speed range [-1000...1000] direction is inverted
static const double max_steering_angle = 180.0f;
static const double min_steering_angle = 0.0f;
static const double treshold_belief_min = 0.7;
static const double treshold_belief_max = 0.9;
static const double treshold_dist_max = 1.0;

class Control {
 public:
  Control(ros::NodeHandle nh):seqNum(0) {
    n_.param<float>("/control_node/dstPosX", dstPosX, 0);
    n_.param<float>("/control_node/dstPosY", dstPosY, 0);
    n_.param<float>("/control_node/max_speed", max_speed, 400.f);

    dstIsSet = false;
    dst.x = dstPosX;
    dst.y = dstPosY;
    dst.z = 0; // because we move on a 2d plane

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
    subDir_ = n_.subscribe("/localization/cps2/particle",1,&Control::setDirection,this); 
    subDst_ = n_.subscribe("/localization/control/dest",1,&Control::setDestination,this);
    pubMotor_ = nh.advertise<std_msgs::Int16>("/manual_control/stop_start", 1);    
    pubSpeed_ = nh.advertise<std_msgs::Int16>("/manual_control/speed", 1);

    msg_stop.data     = 1;
    msg_start.data    = 0;
    // enable motors
    pubMotor_.publish(msg_start);

#ifdef DEBUG_CONTROL
    pubDirectionPose_ = nh.advertise<visualization_msgs::Marker>(nh.resolveName("/localization/control/DirectionPose"), 1);
    pubDst_ = nh.advertise<geometry_msgs::PointStamped>(nh.resolveName("/localization/control/destination"), 1);
#endif
  }
  ~Control(){}

  void setDirection(const cps2_particle_msgs::particle_msgs& msg_particle) {
    // don't aim for no target
    if(!dstIsSet)
      return;

    // check for old message
    if (seqNum < msg_particle.header.seq){        
      seqNum = msg_particle.header.seq;

      // calculate direction
      const float dx         = dstPosX - msg_particle.pose.position.x;
      const float dy         = dstPosY - msg_particle.pose.position.y;
      const float target_yaw = atan2(dy, dx);
      float yaw              = target_yaw - tf::getYaw(msg_particle.pose.orientation);

      if(abs(yaw) > M_PI)
        yaw = (yaw > 0 ? -2 : 2) * M_PI + yaw;

      const float steering   = std::min(180.0, std::max(0.0, yaw * 180 / M_PI + 90) );
      const float distance   = sqrtf(dx * dx + dy * dy);

      steering_angle_msg.data = steering;

      speed_msg.data = -max_speed;

      // scale by belief
      speed_msg.data *=
                msg_particle.belief <= treshold_belief_min ? 0
              : msg_particle.belief >= treshold_belief_max ? 1
              : (msg_particle.belief - treshold_belief_min) / (treshold_belief_max - treshold_belief_min);

      // scale by distance
      if(distance < treshold_dist_max)
        speed_msg.data *= distance / treshold_dist_max;

      pubSpeed_.publish(speed_msg);
      pubSteering_.publish(steering_angle_msg);

#ifdef DEBUG_CONTROL
      dstPoint_msg.header.seq   = msg_particle.header.seq;
      dstPoint_msg.header.stamp = msg_particle.header.stamp;
      dstPoint_msg.point.x      = dst.x;
      dstPoint_msg.point.y      = dst.y;
      pubDst_.publish(dstPoint_msg);

      tf::Quaternion direction_q = tf::createQuaternionFromYaw(target_yaw);

      directionPose_msg.header.seq         = msg_particle.header.seq;
      directionPose_msg.header.stamp       = msg_particle.header.stamp;
      directionPose_msg.pose.position.x    = msg_particle.pose.position.x;
      directionPose_msg.pose.position.y    = msg_particle.pose.position.y;

      directionPose_msg.pose.orientation.x = direction_q.getX();
      directionPose_msg.pose.orientation.y = direction_q.getY();
      directionPose_msg.pose.orientation.z = direction_q.getZ();
      directionPose_msg.pose.orientation.w = direction_q.getW();

      directionPose_msg.scale.x    = distance;//length
        
      pubDirectionPose_.publish(directionPose_msg);
      ROS_INFO("control_node setDirection: pos(%.2f/%.2f) dst(%.2f/%.2f) dist: %.2f speed: %d steering: %.2f belief: %.2f",
               msg_particle.pose.position.x, msg_particle.pose.position.y, dst.x, dst.y, distance, speed_msg.data, steering - 90, msg_particle.belief);
#endif
    }
  }
  
  void setDestination(const geometry_msgs::Point& msg_dst) {
    dstIsSet = true;
    dstPosX = msg_dst.x;
    dstPosY = msg_dst.y;
    dst.x = msg_dst.x;
    dst.y = msg_dst.y;
    
    #ifdef DEBUG_CONTROL
      ROS_INFO("control_node setDestination dst(%.2f/%.2f)", dst.x, dst.y);
    #endif
  }

  bool dstIsSet;
  int seqNum;
  float dstPosX;
  float dstPosY;
  float max_speed;

 protected:
  cv::Point3f dst;
  std_msgs::Int16 speed_msg;
  std_msgs::Int16 steering_angle_msg;
  visualization_msgs::Marker directionPose_msg;
  visualization_msgs::Marker steeringPose_msg;
  geometry_msgs::PointStamped dstPoint_msg;
  ros::NodeHandle n_; 
  ros::Publisher pubSteering_;
  ros::Publisher pubDirectionPose_;
  ros::Publisher pubDst_;
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
  
  ROS_INFO("Control_node: desired:(%.2f,%.2f) max speed: %.2f",
           control.dstPosX, control.dstPosY, control.max_speed);
  
  while(ros::ok()) {
    ros::spin();
  }
  return 0;
}

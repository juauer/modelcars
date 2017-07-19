#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include <opencv2/core/core.hpp>
#include <math.h>
#include <vector>

class PathControl {
 public:
  PathControl(ros::NodeHandle nh): current(0) {
    
    subDst_ = n_.subscribe("/localization/control/destination/reached",1,&PathControl::setDirection,this);

    msg_dst.z = 0;

    dst_list.push_back(cv::Point3f(-0.04,0.03,0.0));
    dst_list.push_back(cv::Point3f(0.1,1.0,0.0));
    dst_list.push_back(cv::Point3f(0.1,0.1,0.0));
    dst_list.push_back(cv::Point3f(0.1,1.0,0.0));
    dst_list.push_back(cv::Point3f(0.1,0.1,0.0));
  }
  ~PathControl(){}

  void setDirection(const std_msgs::Bool& msg_dst_reached) {
    msg_dst.x = dst_list[current].x;
    msg_dst.y = dst_list[current].y;

    current = (current + 1) % dst_list.size();
  }
  
 protected:
  ros::Publisher pubDst_;
  geometry_msgs::Point msg_dst;
  unsigned current;
  std::vector<cv::Point3f> dst_list;
  ros::Subscriber subDst_;
  ros::NodeHandle n_; 

};//End of class auto_stop

int main(int argc, char **argv) {
  ros::init(argc, argv, "path_control_node");
  ros::NodeHandle nh; 
  PathControl path(nh);
  
  ROS_INFO("Path_Control_node initialization");
  
  while(ros::ok()) {
    ros::spin();
  }
  return 0;
}

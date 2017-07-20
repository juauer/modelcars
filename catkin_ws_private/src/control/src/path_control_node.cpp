#include <ros/ros.h>
#include <ros/package.h>
#include <tf/tf.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include <opencv2/core/core.hpp>
#include <math.h>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>

class PathControl {
 public:
  PathControl(ros::NodeHandle nh): current(0) {
    n_.param<std::string>("pathFile", pathFile, "track01.csv");
    
    subDst_ = n_.subscribe("/localization/control/destination/reached",1,&PathControl::setDestination,this);
    pubDst_ = nh.advertise<geometry_msgs::Point>(nh.resolveName("/localization/cps2/dst"), 1);

    msg_dst.z = 0;

    std::string path = ros::package::getPath("control") + std::string("/config/") + pathFile;
    
    std::ifstream file;
    std::string line_str;

    float _x = 0, _y = 0;
    
    file.open(path.c_str(), std::ios::in);
    while(file.good()){
      getline(file, line_str);
      std::stringstream line_ss(line_str);

      line_ss >> _x >> _y;
      ROS_INFO("Path_Control_node point %lu (%.2f/%.2f)", point_list.size(), _x, _y);

      point_list.push_back(cv::Point3f(_x, _y, 0.0));
    }
    file.close();
    ROS_INFO("Path_Control_node initialization Num Points: %lu path: %s", point_list.size(), path.c_str());
  }
  ~PathControl(){}

  void setDestination(const std_msgs::Bool& msg_dst_reached) {
    msg_dst.x = point_list[current].x;
    msg_dst.y = point_list[current].y;

    current = (current + 1) % point_list.size();
    pubDst_.publish(msg_dst);
    ROS_INFO("Path_Control_node setDestination current: %d dst(%.2f/%.2f)", current, msg_dst.x, msg_dst.y);
  }

  std::string pathFile;  
 protected:
  ros::Publisher pubDst_;
  geometry_msgs::Point msg_dst;
  unsigned current;
  std::vector<cv::Point3f> point_list;
  ros::Subscriber subDst_;
  ros::NodeHandle n_; 

};//End of class auto_stop

int main(int argc, char **argv) {
  ros::init(argc, argv, "path_control_node");
  ros::NodeHandle nh; 
  PathControl path(nh);
  
  ROS_INFO("Path_Control_node initialization pathfile: %s", path.pathFile.c_str());
  
  while(ros::ok()) {
    ros::spin();
  }
  return 0;
}

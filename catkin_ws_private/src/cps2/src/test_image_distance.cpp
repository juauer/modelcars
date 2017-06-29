#include <opencv2/highgui.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include "map.hpp"

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "stitching");
	ros::NodeHandle nh;

	std::string path = ros::package::getPath("cps2") + std::string("/../../../captures/");
	
	std::string path_img = path + std::string(argv[1]);
	if(access(path_img.c_str(), F_OK ) == -1)
	{
		ROS_ERROR("No such file: %s\nPlease give a path relative to modelcars/captures/.\n", path_img.c_str() );
		return 1;
	}
	cv::Mat img1 = cv::imread(path_img);
	
	path_img = path + std::string(argv[2]);
	if(access(path_img.c_str(), F_OK ) == -1)
	{
		ROS_ERROR("No such file: %s\nPlease give a path relative to modelcars/captures/.\n", path_img.c_str() );
		return 1;
	}
	cv::Mat img2 = cv::imread(path_img);
	
	cv::Point3f vector;
	vector.x = atof(argv[3]);
	vector.y = atof(argv[4]);
	vector.z = atof(argv[5]);
	
	cv::Point3f flow = cps2::Map::image_distance(img1, img2, vector);
	
	printf("--best vector = %f, %f, %f \n", flow.x, flow.y, flow.z);

	// TODO imshow the images and draw the resulting vector on one of them

	return 0;
}

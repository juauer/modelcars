#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include "../map.hpp"

cps2::ImageEvaluator *image_evaluator;
cps2::Map *map;

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "stitching");
	ros::NodeHandle nh;
	
	image_evaluator = new cps2::ImageEvaluator(cps2::IE_MODE_PIXELS, 1, 1, 1);
	map = new cps2::Map(image_evaluator, 1.0, 1, 60);

	std::string path = ros::package::getPath("cps2") + std::string("/../../../captures/");
	
	std::string path_img = path + std::string(argv[1]);
	if(access(path_img.c_str(), F_OK ) == -1)
	{
		ROS_ERROR("No such file: %s\nPlease give a path relative to modelcars/captures/.\n", path_img.c_str() );
		return 1;
	}
	cv::Mat img1;
	cv::cvtColor(cv::imread(path_img), img1, CV_BGR2GRAY);
	
	path_img = path + std::string(argv[2]);
	if(access(path_img.c_str(), F_OK ) == -1)
	{
		ROS_ERROR("No such file: %s\nPlease give a path relative to modelcars/captures/.\n", path_img.c_str() );
		return 1;
	}
	cv::Mat img2;
	cv::cvtColor(cv::imread(path_img), img2, CV_BGR2GRAY);
	
	cv::Point3f pos_prev;
	pos_prev.x = atof(argv[3]);
	pos_prev.y = atof(argv[4]);
	pos_prev.z = atof(argv[5]);
	
	cv::Point3f pos_new;
	pos_new.x = atof(argv[6]);
	pos_new.y = atof(argv[7]);
	pos_new.z = atof(argv[8]);
	
	cv::Point3f flow = map->image_distance(img1, img2, pos_prev, pos_new);
	
	printf("--best vector in world frame = %f, %f, %f \n", flow.x, flow.y, flow.z);
	
	// vector relative to img1 rotation
	cv::Point2f pos_rel2f;
	pos_rel2f.x = cosf(-pos_prev.z)*flow.x - sinf(-pos_prev.z)*flow.y;
	pos_rel2f.y = sinf(-pos_prev.z)*flow.x + cosf(-pos_prev.z)*flow.y;
	
	fisheye_camera_matrix::CameraMatrix cm(640, 480, 0, 0, 320, 2.55, 1.5);
	
	cv::Point2i pos_image = cm.relative2image(pos_rel2f);
	cv::line(img1, cv::Point2i(img1.cols/2,img1.rows/2), cv::Point2i(img1.cols/2,img1.rows/2)+pos_image, 0,1,8,0);
	printf("dx: %i, dy: %i\n",pos_image.x, pos_image.y);

	cv::imshow("Bild 1",img1);
	cv::imshow("Bild 2",img2);
	cv::waitKey(0);

	
	return 0;
}

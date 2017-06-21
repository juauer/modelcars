#include <stdio.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>

cv::Point3f correct_position(cv::Mat img1, cv::Mat img2, cv::Point3f flow_est) {
	// flow_est in Bild-Koordinaten von img1 nach img2: x- Pixel nach rechts, y-Pixel nach oben theta- Drehung im Uhrzeigersinn
	int x = (int)flow_est.x;
	int y = (int)flow_est.y;
	int theta = (int)flow_est.z;
	
	cv::Mat gray1, gray2;
	cv::cvtColor(img1, gray1, CV_BGR2GRAY);
	cv::cvtColor(img2, gray2, CV_BGR2GRAY);
	
	//cut fisheye projection
	cv::Mat i1(gray1, cv::Rect(34, 56, img1.cols-68, img1.rows-112));
	cv::Mat i2(gray2, cv::Rect(34, 56, img1.cols-68, img1.rows-112));
	
	cv::Point3f flow_corr = flow_est;
	float min_error = 1000;
	
	for (int dtheta=-5; dtheta < 6; ++dtheta)
	{
		cv::Mat r = cv::getRotationMatrix2D(cv::Point2f(i2.cols/2., i2.rows/2.), theta+dtheta, 1.0);
		cv::Mat img2_rot;
		cv::warpAffine(i2, img2_rot, r, cv::Size(i2.cols, i2.rows));
		
		for (int dx=-10; dx < 11; ++dx)
		{
			for (int dy=-10; dy < 11; ++dy)
			{
				cv::Mat part1(i1, cv::Rect(std::max(x+dx, 0), std::max(y+dy, 0), i1.cols-abs(x+dx), i1.rows-abs(y+dy)));
				cv::Mat part2(img2_rot, cv::Rect(std::max(-x-dx, 0), std::max(-y-dy, 0), i1.cols-abs(x+dx), i1.rows-abs(y+dy)));

				cv::Mat tile1, tile2;
				part1.copyTo(tile1, part2);
				part2.copyTo(tile2, part1);

				cv::Mat test = tile1 - tile2;
				if (min_error > cv::sum(cv::mean(test))[0])
				{
					min_error = cv::sum(cv::mean(test))[0];
					flow_corr = cv::Point3f(x+dx, y+dy, theta+dtheta);
				}
			}
		}
	}
	
	return flow_corr;
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "stitching");
	ros::NodeHandle nh;

	std::string path = ros::package::getPath("stitching") + std::string("/../../../captures/");
	
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
	
	cv::Point3f flow = correct_position(img1, img2, vector);
	
	//printf("--best vector = %f, %f, %f \n", flow.x, flow.y, flow.z);
	return 0;
}

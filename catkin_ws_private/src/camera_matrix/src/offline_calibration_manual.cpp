#include <string>
#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <opencv2/highgui.hpp>

#include "camera_matrix.hpp"

struct Intrinsics {
	int cx;
	int cy;
	int flx;
	int fly;
};

struct DistortionCoefs {
	int dcrx;
	int dcry;
	int dctx;
	int dcty;
};

cv::Mat img;
cv::Mat img_undistorted;

Intrinsics int_val;
Intrinsics int_max;
DistortionCoefs dist_val;
DistortionCoefs dist_max;

void apply(int, void*) {
	CameraMatrix cm(int_val.cx, int_val.cy, int_val.flx, int_val.fly,
			dist_val.dcrx / 10 - 4, dist_val.dcry / 10 - 4,
			dist_val.dctx / 10 - 4, dist_val.dcty / 10 - 4);

	cm.undistort(img, img_undistorted);
	cv::imshow("undistorted", img_undistorted);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "calibration");

	if(argc < 3) {
		ROS_ERROR("Missing argument(s).");
		return 1;
	}

	std::string path_img   = ros::package::getPath("camera_matrix") + std::string("/../../../captures/") + std::string(argv[1]);
	std::string path_calib = ros::package::getPath("camera_matrix") + std::string("/config/") + std::string(argv[2]);

	if(access(path_img.c_str(), F_OK ) == -1) {
		ROS_ERROR("No such file: %s\nPlease give a path relative to catkin_ws/../captures/.\n", path_img.c_str() );
		return 1;
	}

	img = cv::imread(path_img);

	int width  = img.cols;
	int height = img.rows;

	int_val.cx    = width / 2;
	int_val.cy    = height / 2;
	int_val.flx   = 2 * width;
	int_val.fly   = 2 * width;
	dist_val.dcrx = 40;
	dist_val.dcry = 40;
	dist_val.dctx = 40;
	dist_val.dcty = 40;
	int_max.cx    = width;
	int_max.cy    = height;
	int_max.flx   = 3 * width;
	int_max.fly   = 3 * width;
	dist_max.dcrx = 80;
	dist_max.dcry = 80;
	dist_max.dctx = 80;
	dist_max.dcty = 80;

	cv::namedWindow("undistorted", CV_WINDOW_AUTOSIZE);
	cv::createTrackbar("cx",   "undistorted", &int_val.cx,    int_max.cx,    apply);
	cv::createTrackbar("cy",   "undistorted", &int_val.cy,    int_max.cy,    apply);
	cv::createTrackbar("flx",  "undistorted", &int_val.flx,   int_max.flx,   apply);
	cv::createTrackbar("fly",  "undistorted", &int_val.fly,   int_max.fly,   apply);
	cv::createTrackbar("dcrx", "undistorted", &dist_val.dcrx, dist_max.dcrx, apply);
	cv::createTrackbar("dcry", "undistorted", &dist_val.dcry, dist_max.dcry, apply);
	cv::createTrackbar("dctx", "undistorted", &dist_val.dctx, dist_max.dctx, apply);
	cv::createTrackbar("dcty", "undistorted", &dist_val.dcty, dist_max.dcty, apply);
	cv::imshow("image", img);
	apply(0, (void*)0);
	cv::waitKey(0);

	std::ofstream file;
	file.open(path_calib.c_str(), std::ios::out | std::ios::trunc);

	file << int_val.cx << " " << int_val.cy << " " << int_val.flx << " " << int_val.fly << " "
			<< dist_val.dcrx / 10 - 4 << " " << dist_val.dcry / 10 - 4
			<< " " << dist_val.dctx / 10 - 4 << " " << dist_val.dcty / 10 - 4 << std::endl;

	file.close();
	ROS_INFO("Calibration written to %s.", path_calib.c_str() );
	return 0;
}

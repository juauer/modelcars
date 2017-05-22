#include <stdio.h>
#include <string>
#include <ros/package.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include "image_evaluator.hpp"

cps2::Map *map;
cv::Mat img;

int px = 100;
int py = 75;
int pt = 0;
int sc = 3;
int sz = 5;
int de = 2;

const int max_px = 640;
const int max_py = 480;
const int max_pt = 8;
const int max_sc = 160;
const int max_sz = 21;
const int max_de = 10;

void apply(int, void*) {
	if(px == 0 || py == 0 || sc == 0 || sz == 0 || de == 0)
		return;

	cv::Point3f p(px, py, pt * M_PI / 4);
	cps2::ImageEvaluator(*map, sc, sz, de).evaluate(img, p);
}

int main(int, char**) {
	std::string path = ros::package::getPath("cps2") + std::string("/../../../captures/(200,200,0).jpg");

	map = new cps2::Map(path.c_str() );
	img = cv::Mat(map->img_bgr, cv::Rect(0, 100, 200, 150) );

	cv::namedWindow("test", CV_WINDOW_AUTOSIZE);
	cv::createTrackbar("particle x   ", "test", &px, max_px, apply);
	cv::createTrackbar("particle y   ", "test", &py, max_py, apply);
	cv::createTrackbar("particle th  ", "test", &pt, max_pt, apply);
	cv::createTrackbar("downscale    ", "test", &sc, max_sc, apply);
	cv::createTrackbar("kernel size  ", "test", &sz, max_sz, apply);
	cv::createTrackbar("std deviation", "test", &de, max_de, apply);

	apply(0, (void*)0);
	cv::waitKey(0);
}

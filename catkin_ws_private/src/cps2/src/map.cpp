#include <math.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "map.hpp"

cps2::Map::Map(const char *path) {
	img_bgr = cv::imread(path);

	cv::cvtColor(img_bgr, img_gray, CV_BGR2GRAY);
}

cps2::Map::~Map() {

}

cv::Point3f cps2::Map::map2world(cv::Point3f &pose_map) {

	// TODO remove (guessed!) magic numbers

	cv::Point3f p;
	p.x = 0.01 * pose_map.x;
	p.y = 0.01 * (img_gray.rows - pose_map.y);
	p.z = pose_map.z + M_PI / 2;
	return p;
}

cv::Point3f cps2::Map::world2map(cv::Point3f &pose_world) {

	// TODO remove (guessed!) magic numbers

	cv::Point3f p;
	p.x = 100 * pose_world.x;
	p.y = img_gray.rows - 100 * pose_world.y;
	p.z = pose_world.z - M_PI / 2;
	return p;
}

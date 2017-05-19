#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "map.hpp"

cps2::Map::Map(const char *path) {
	img_bgr = cv::imread(path);

	cv::cvtColor(img_bgr, img_gray, CV_BGR2GRAY);
}

cps2::Map::~Map() {

}

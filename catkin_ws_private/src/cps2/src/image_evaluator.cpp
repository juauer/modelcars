#include <math.h>

#include "image_evaluator.hpp"

// ---- test ----
#include <string>
#include <ros/package.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
// --------------

namespace cps2 {

ImageEvaluator::ImageEvaluator(Map &_map) : map(_map) {

}

ImageEvaluator::~ImageEvaluator() {

}

float ImageEvaluator::evaluate(cv::Mat &img, cv::Point3f &particle) {
	int piece_rows = img.rows / resize_scale;
	int piece_cols = img.cols / resize_scale;
	int piece_cx   = piece_cols / 2;
	int piece_cy   = piece_rows / 2;

	cv::Mat piece(piece_rows, piece_cols, CV_8UC1);

	for(int r = 0; r < piece_rows; ++r)
		for(int c = 0; c < piece_cols; ++c) {
			int sx  = c - piece_cx;
			int sy  = r - piece_cy;
			float x = resize_scale * (sx * cos(particle.z) - sy * sin(particle.z) ) + particle.x;
			float y = resize_scale * (sx * sin(particle.z) + sy * cos(particle.z) ) + particle.y;

			if(x >= 0 && y >= 0 && x < map.img_gray.cols && y < map.img_gray.rows)
				piece.at<uchar>(r, c) = map.img_gray.at<uchar>( (int)y, (int)x);
			else
				piece.at<uchar>(r, c) = 0;
		}

// ---- test ----
std::string path = ros::package::getPath("cps2") + std::string("/../../../captures/");
cv::imwrite(path + std::string("test_ie_01_map.jpg"), map.img_gray);
cv::imwrite(path + std::string("test_ie_02_subimage.jpg"), img);
cv::imwrite(path + std::string("test_ie_03_scaled_and_rotated.jpg"), piece);
// --------------

	return 0.0;
}

} /* namespace cps2 */




int main(int, char**) {
	std::string path = ros::package::getPath("cps2") + std::string("/../../../captures/(200,200,0).jpg");
	cps2::Map map(path.c_str() );
	cps2::ImageEvaluator ie(map);

	cv::Mat i(map.img_gray, cv::Rect(0, 0, 440, 280) );
	cv::Point3f p(220, 140, 0.7);
	ie.evaluate(i, p);
}

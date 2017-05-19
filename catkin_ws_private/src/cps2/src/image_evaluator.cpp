#include "image_evaluator.hpp"

namespace cps2 {

ImageEvaluator::ImageEvaluator(Map &_map) : map(_map) {

}

ImageEvaluator::~ImageEvaluator() {

}

float ImageEvaluator::evaluate(cv::Mat &img, cv::Point3f &particle) {
	cv::Mat piece(img.rows / resize_scale, img.cols / resize_scale, CV_8UC3);

	for(int r=0; r<piece.rows; ++r)
		for(int c=0; c<piece.cols; ++c) {
			int x = particle.x + c * resize_scale;
			piece.at<uchar>(r, c) = map.img.at<uchar>();
		}

	return 0.0;
}

} /* namespace cps2 */

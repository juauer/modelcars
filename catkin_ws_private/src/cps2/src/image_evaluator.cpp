#include <math.h>
#include <opencv2/imgproc.hpp>

#ifdef DEBUG_IE
#include <stdio.h>
#include <opencv2/highgui.hpp>
#endif

#include "image_evaluator.hpp"

namespace cps2 {

#ifdef DEBUG_IE
int d_img_width  = 360;
int d_img_height = 240;
cv::Size d_win_size(d_img_width, d_img_height);
cv::Mat d_win_img(d_img_height, 2 * d_img_width, CV_8UC1);
#endif

void ImageEvaluator::generateKernel() {
	int center = kernel_size / 2;
	float acc  = 0;

	cv::Mat kernel(kernel_size, kernel_size, CV_32FC1);

	for(int r = 0; r < kernel_size; ++r) {
		int y = r - center;

		for(int c = 0; c < kernel_size; ++c) {
			int x = c - center;
			kernel.at<float>(r, c) = exp( (-x*x - y*y) / (2 * kernel_stddev*kernel_stddev) );
			acc += kernel.at<float>(r, c);
		}
	}

	this->kernel = kernel;
}

int ImageEvaluator::applyKernel(cv::Mat &img, int x, int y) {
	int ks2     = kernel_size / 2;
	int x_lb    = std::max(0, x - ks2);
	int x_ub    = std::min(img.cols, x + ks2 + 1);
	int y_lb    = std::max(0, y - ks2);
	int y_ub    = std::min(img.rows, y + ks2 + 1);
	float acc_k = 0;
	float acc_v = 0;

	for(int r = y_lb; r < y_ub; ++r) {
		int ky = r - y + ks2;

		for(int c = x_lb; c < x_ub; ++c) {
			int kx = c - x + ks2;

			acc_k += kernel.at<float>(ky, kx);
			acc_v += kernel.at<float>(ky, kx) * img.at<uchar>(r, c);
		}
	}

	return (int)(acc_v / acc_k);
}

ImageEvaluator::ImageEvaluator(Map &_map, int _resize_scale, int _kernal_size, float _kernal_stddev) :
		map(_map),
		resize_scale(_resize_scale),
		kernel_stddev(_kernal_stddev)
{
	kernel_size = 2 * (_kernal_size / 2) + 1;
	generateKernel();
}

ImageEvaluator::ImageEvaluator(Map &_map) : map(_map) {
	resize_scale  = 20;
	kernel_size   = 3;
	kernel_stddev = 1;

	generateKernel();
}

ImageEvaluator::~ImageEvaluator() {

}

float ImageEvaluator::evaluate(cv::Mat &img, cv::Point3f &particle) {
	int dim_y = img.rows / resize_scale;
	int dim_x = img.cols / resize_scale;
	int cx    = dim_x / 2;
	int cy    = dim_y / 2;

	cv::Mat mappiece(dim_y, dim_x, CV_8UC1);

	for(int r = 0; r < dim_y; ++r)
		for(int c = 0; c < dim_x; ++c) {
			int sx  = c - cx;
			int sy  = r - cy;
			float x = resize_scale * (sx * cos(particle.z) - sy * sin(particle.z) ) + particle.x;
			float y = resize_scale * (sx * sin(particle.z) + sy * cos(particle.z) ) + particle.y;

			if(x >= 0 && y >= 0 && x < map.img_gray.cols && y < map.img_gray.rows)
				mappiece.at<uchar>(r, c) = applyKernel(map.img_gray, (int)x, (int)y);
			else
				mappiece.at<uchar>(r, c) = 0;
		}

	cv::Mat img_gray;
	cv::Mat img_tf(dim_y, dim_x, CV_8UC1);

	cv::cvtColor(img, img_gray, CV_BGR2GRAY);

	for(int r = 0; r < dim_y; ++r)
		for(int c = 0; c < dim_x; ++c)
			img_tf.at<uchar>(r, c) = applyKernel(img_gray, resize_scale * c, resize_scale * r);

	float result = 0;
	int pixels   = dim_x * dim_y;

	for(int r = 0; r < dim_y; ++r)
		for(int c = 0; c < dim_x; ++c)
			if(mappiece.at<uchar>(r, c) == 0)
				--pixels;
			else
				result += abs(mappiece.at<uchar>(r, c) - img_tf.at<uchar>(r, c));

	result /= 255 * pixels;

#ifdef DEBUG_IE
cv::resize(img_tf, d_win_img(cv::Rect(0 ,0, 360, 240) ), d_win_size);
cv::resize(mappiece, d_win_img(cv::Rect(360 ,0, 360, 240) ), d_win_size );
cv::imshow("test", d_win_img);
#endif

	return result;
}

} /* namespace cps2 */
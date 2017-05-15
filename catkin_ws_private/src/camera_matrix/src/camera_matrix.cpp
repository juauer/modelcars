#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

#include "camera_matrix.hpp"

camera_matrix_msgs::CameraMatrix msg2;

CameraMatrix::CameraMatrix(float cx, float cy, float flx, float fly,
			float dcrx, float dcry, float dctx, float dcty) {
	K = (cv::Mat_<float>(3, 3) <<
			flx,   0, cx,
			  0, fly, cy,
			  0,   0,  1);

	D = (cv::Mat_<float>(4, 1) << dcrx, dcry, dctx, dcty);
}

CameraMatrix::CameraMatrix() {
	K = (cv::Mat_<float>(3, 3) <<
				1, 0, 1,
				0, 1, 1,
				0, 0, 1);

	D = (cv::Mat_<float>(4, 1) << 0, 0, 0, 0);
}

CameraMatrix::CameraMatrix(char* config=NULL) {

}

CameraMatrix::CameraMatrix(camera_matrix_msgs::CameraMatrix msg) {
	K = (cv::Mat_<float>(3, 3) <<
				msg.flx,   0, cx,
				  0, fly, cy,
				  0,   0,  1);

		D = (cv::Mat_<float>(4, 1) << dcrx, dcry, dctx, dcty);
}

CameraMatrix::~CameraMatrix() {

}

camera_matrix_msgs::CameraMatrix CameraMatrix::serialize() {
	return msg2;
}

bool CameraMatrix::calibrate_intrinsics_offline_manual(char** config) {
	return false;
}

bool CameraMatrix::calibrate_intrinsics_offline_auto(char** config) {
	return false;
}

bool CameraMatrix::set_extrinsics(char** config) {
	return false;
}

bool CameraMatrix::calibrate_online_update() {
	return false;
}

cv::Mat CameraMatrix::undistort(cv::Mat &img) {
	cv::Mat dst;
	cv::fisheye::undistortImage(img, dst, K, D, K);
	return dst;
}

cv::Point2f CameraMatrix::inverse_perspective_tf(cv::Point &p) {
	return cv::Point2f();
}

cv::Mat img = cv::imread("../captures/test.jpg");
cv::Mat ud;

int const width  = img.cols;
int const height = img.rows;

int cx  = width / 2;
int cy  = height / 2;
int flx = 2 * width;
int fly = 2 * width;

int dcrx = 40;
int dcry = 40;
int dctx = 40;
int dcty = 40;

int const cx_max   = width;
int const cy_max   = height;
int const flx_max  = 3 * width;
int const fly_max  = 3 * width;
int const dcrx_max = 80;
int const dcry_max = 80;
int const dctx_max = 80;
int const dcty_max = 80;

void apply(int, void*) {
	cv::Mat K = (cv::Mat_<float>(3, 3) <<
			flx,   0, cx,
			  0, fly, cy,
			  0,   0,  1);

	cv::Mat D = (cv::Mat_<float>(4, 1) << dcrx / 10.0 - 4, dcry / 10.0 - 4, dctx / 10.0 - 4, dcty / 10.0 - 4);

	cv::imshow("ud", ud);
}
/*
int main(int, char**) {
	cv::namedWindow("ud", CV_WINDOW_AUTOSIZE);
	cv::createTrackbar("cx",   "ud", &cx,   cx_max,   apply);
	cv::createTrackbar("cy",   "ud", &cy,   cy_max,   apply);
	cv::createTrackbar("flx",  "ud", &flx,  flx_max,  apply);
	cv::createTrackbar("fly",  "ud", &fly,  fly_max,  apply);
	cv::createTrackbar("dcrx", "ud", &dcrx, dcrx_max, apply);
	cv::createTrackbar("dcry", "ud", &dcry, dcry_max, apply);
	cv::createTrackbar("dctx", "ud", &dctx, dctx_max, apply);
	cv::createTrackbar("dcty", "ud", &dcty, dcty_max, apply);
	cv::imshow("img", img);
	apply(0,(void*)0);
	cv::waitKey(0);
	return 0;
}*/

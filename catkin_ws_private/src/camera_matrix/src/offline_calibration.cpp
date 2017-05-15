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

	img_undistorted = cm.undistort(img);
	cv::imshow("undistorted", img_undistorted);
}

int main(int, char**) {

	// TODO add switch for auto calibration

	// TODO parse path from cl

	img = cv::imread("../captures/test.jpg");

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

	// TODO save to config/

	return 0;
}

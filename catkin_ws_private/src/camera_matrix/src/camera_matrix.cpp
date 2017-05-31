#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

#include "camera_matrix.hpp"

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

CameraMatrix::CameraMatrix(const char *path) {
  std::ifstream file;
  std::string line_str;
  float cx, cy, flx, fly, dcrx, dcry, dctx, dcty;

  file.open(path, std::ios::in);
  getline(file, line_str);

  std::stringstream line_ss(line_str);

  line_ss >> cx >> cy >> flx >> fly >> dcrx >> dcry >> dctx >> dcty;

  file.close();

  K = (cv::Mat_<float>(3, 3) <<
          flx,   0, cx,
            0, fly, cy,
            0,   0,  1);

  D = (cv::Mat_<float>(4, 1) << dcrx, dcry, dctx, dcty);
}

CameraMatrix::CameraMatrix(camera_matrix_msgs::CameraMatrix msg) {
  K = (cv::Mat_<float>(3, 3) <<
        msg.flx,       0, msg.cx,
              0, msg.fly, msg.cy,
              0,       0,      1);

  D = (cv::Mat_<float>(4, 1) << msg.dcrx, msg.dcry, msg.dctx, msg.dcty);
}

CameraMatrix::~CameraMatrix() {

}

camera_matrix_msgs::CameraMatrix CameraMatrix::serialize() {
  camera_matrix_msgs::CameraMatrix msg;

  msg.cx   = K.at<float>(0, 2);
  msg.cy   = K.at<float>(1, 2);
  msg.flx  = K.at<float>(0, 0);
  msg.fly  = K.at<float>(1, 1);
  msg.dcrx = D.at<float>(0, 0);
  msg.dcry = D.at<float>(1, 0);
  msg.dctx = D.at<float>(2, 0);
  msg.dcty = D.at<float>(3, 0);

  return msg;
}

bool CameraMatrix::update_calibration() {

  // TODO implement automatic online calibration

  return false;
}

cv::Mat CameraMatrix::undistort(cv::Mat &src, cv::Mat &dst) {
  cv::fisheye::undistortImage(src, dst, K, D, K);
  return dst;
}

cv::Point2f CameraMatrix::inverse_perspective_tf(cv::Point &p) {

  // TODO transform p (image coords) to world coords

  return cv::Point2f();
}

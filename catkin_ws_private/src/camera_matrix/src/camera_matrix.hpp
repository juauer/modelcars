#include <opencv2/core.hpp>

#include "camera_matrix_msgs/CameraMatrix.h"

class CameraMatrix {
public:
  CameraMatrix(float cx, float cy, float flx, float fly,
      float dcrx, float dcry, float dctx, float dcty);
  CameraMatrix();
  CameraMatrix(const char *path);
  CameraMatrix(camera_matrix_msgs::CameraMatrix);
  ~CameraMatrix();

  camera_matrix_msgs::CameraMatrix serialize();

  bool update_calibration();

  cv::Mat undistort(cv::Mat &src, cv::Mat &dst);
  cv::Point2f inverse_perspective_tf(cv::Point &p);

private:
  cv::Mat K;
  cv::Mat D;
};

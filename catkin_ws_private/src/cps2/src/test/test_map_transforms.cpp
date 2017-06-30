#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <ros/package.h>
#include "fisheye_camera_matrix/camera_matrix.hpp"
#include "../image_evaluator.hpp"

int px = 0;
int py = 0;
int pt = 0;
int mx = 0;
int my = 0;
int mt = 0;
int pxm = 5;
int pym = 5;
int ptm = 8;
int mxm = 5;
int mym = 5;
int mtm = 8;

cv::Mat img;
fisheye_camera_matrix::CameraMatrix *camera_matrix;
cps2::ImageEvaluator *evaluator;

void apply(int, void *) {
  cv::Point3f pos_world(px, py, pt * M_PI / 4 );
  cv::Point3f   pos_map(mx, my, mt * M_PI / 4 );

  cv::Point2f pos_rel(
      pos_world.x - pos_map.x,
      -pos_world.y + pos_map.y
  );

  cv::Point2i pos_image = camera_matrix->relative2image(pos_rel);

  cv::Mat r = evaluator->transform(img, pos_image, pos_world.z, -pos_map.z);
  cv::imshow("test", r);
}

int main(int, char**) {
  img = cv::imread(
      ros::package::getPath("cps2") + std::string("/../../../captures/test_ie.jpg")
  );

  camera_matrix = new fisheye_camera_matrix::CameraMatrix(
      img.cols, img.rows, img.cols / 2, img.rows / 2, img.cols / 2, 2.5, 1);

  evaluator = new cps2::ImageEvaluator(cps2::IE_MODE_PIXELS, 1, 1, 1);

  cv::cvtColor(img, img, CV_BGR2GRAY);
  cv::namedWindow("test", CV_WINDOW_AUTOSIZE);
  cv::createTrackbar("particle x", "test", &px, pxm, apply);
  cv::createTrackbar("particle y", "test", &py, pym, apply);
  cv::createTrackbar("particle t", "test", &pt, ptm, apply);
  cv::createTrackbar("map x", "test", &mx, mxm, apply);
  cv::createTrackbar("map y", "test", &my, mym, apply);
  cv::createTrackbar("map t", "test", &mt, mtm, apply);
  cv::waitKey(0);

  return 0;
}

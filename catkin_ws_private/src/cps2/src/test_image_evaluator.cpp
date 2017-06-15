#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <ros/package.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include "map.hpp"

std::string path;
cv::Rect2i roi;
cv::Mat img;

int px = 100;
int py = 75;
int pt = 0;
int sc = 3;
int sz = 5;
int de = 2;

const int max_px = 640;
const int max_py = 480;
const int max_pt = 8;
const int max_sc = 160;
const int max_sz = 21;
const int max_de = 10;

void apply(int, void *) {
  if (px == 0 || py == 0 || sc == 0 || sz == 0 || de == 0)
    return;

  cv::Point3f p(px, py, pt * M_PI / 4);
  cps2::ImageEvaluator evaluator(cps2::IE_MODE_PIXELS, sc, sz, de);
  cps2::Map map(path.c_str(), &evaluator);

  // TODO soon theOnePice will be gone. what then?

  cv::Mat img1 = evaluator.transform(map.theOnePiece.img, p, cv::Size2i(roi.width, roi.height) );
  cv::Mat img2 = evaluator.transform(img,
      cv::Point3f(img.cols / 2, img.rows / 2, 0), cv::Size2i(img.cols, img.rows) );

  evaluator.evaluate(img1, img2);
}

int main(int argc, char **argv) {
  if (argc < 5)
    printf("Please use roslaunch: 'roslaunch cps2 test_image_evaluator.launch "
           "[x:=INT] [y:=INT] [w:=INT] [h:=INT]'\n");

  path = ros::package::getPath("cps2") + std::string("/../../../captures/map_test.png");
  roi  = cv::Rect2i(atoi(argv[1]), atoi(argv[2]), atoi(argv[3]), atoi(argv[4]) );

  printf("test_image_evaluator: Using x: %d, y: %d, w: %d, h: %d\n",
      atoi(argv[1]), atoi(argv[2]), atoi(argv[3]), atoi(argv[4]) );

  cps2::ImageEvaluator evaluator(cps2::IE_MODE_PIXELS);
  cps2::Map map(path.c_str(), &evaluator);

  img = cv::Mat(map.theOnePiece.img, roi);

  cv::namedWindow("test", CV_WINDOW_AUTOSIZE);
  cv::createTrackbar("particle x   ", "test", &px, max_px, apply);
  cv::createTrackbar("particle y   ", "test", &py, max_py, apply);
  cv::createTrackbar("particle th  ", "test", &pt, max_pt, apply);
  cv::createTrackbar("downscale    ", "test", &sc, max_sc, apply);
  cv::createTrackbar("kernel size  ", "test", &sz, max_sz, apply);
  cv::createTrackbar("std deviation", "test", &de, max_de, apply);

  apply(0, (void *)0);
  cv::waitKey(0);
}

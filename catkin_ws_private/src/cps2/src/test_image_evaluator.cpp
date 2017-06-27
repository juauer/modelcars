#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <ros/package.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include "map.hpp"

cv::Mat img;

int px = 0;
int py = 0;
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
  if (sc == 0 || sz == 0 || de == 0)
    return;

  cps2::ImageEvaluator evaluator(cps2::IE_MODE_PIXELS, sc, sz, de);

  cv::Mat img1 = evaluator.transform(img, cv::Point3f(img.cols / 2 + px, img.rows / 2 + py, pt * M_PI / 4) );
  cv::Mat img2 = evaluator.transform(img, cv::Point3f(img.cols / 2, img.rows / 2, 0) );

  evaluator.evaluate(img1, img2);
}

int main(int argc, char **argv) {
  img = cv::imread(
      ros::package::getPath("cps2") + std::string("/../../../captures/test_ie.jpg")
  );

  cv::cvtColor(img, img, CV_BGR2GRAY);

  cps2::ImageEvaluator evaluator(cps2::IE_MODE_PIXELS, 1, 1, 1);

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

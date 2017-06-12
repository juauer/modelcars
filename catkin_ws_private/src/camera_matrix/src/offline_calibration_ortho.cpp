#include <string>
#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <opencv2/highgui.hpp>
#include <math.h>
#include "camera_matrix.hpp"
#include <opencv2/imgproc.hpp>
int w;
int h;
int cx;
int cy;
int flx = 320;
int flx_max = 640;
int wu;
int hu;
int cxu;
int cyu;

cv::Mat img;

int ch = 200; // cam (not ground) bis ceil
int ch_max = 300;
int sc = 10;
int sc_max = 30;

cv::Point2f c2i(cv::Point2f cc) {
  cv::Point2f c(-cc.y, cc.x);
  float ceilh=ch/100.0;
  return cv::Point2f(cxu+(10.0/sc)*flx * c.x / ceilh, cyu+(10.0/sc)*flx * c.y / ceilh);
}

cv::Point2f i2c(cv::Point2f i) {
  float ceilh=ch/100.0;
  cv::Point2f ii((i.x-cxu) * ceilh / flx, (i.y-cyu) * ceilh / flx);
  return cv::Point2f((0.1*sc)*ii.y, -(0.1*sc)*ii.x);
}

void apply(int, void*) {
  cv::Mat img_undistorted(img.rows*1.5, 1.5*img.cols, CV_8UC3);

  wu = img_undistorted.cols;
  hu = img_undistorted.rows;
  cxu = wu / 2;
  cyu = hu / 2;

  for(float x = 0; x<wu; x+=1)
    for(float y=0;y<hu;y+=1) {
      img_undistorted.at<cv::Vec3b>((int)y,(int)x) = cv::Vec3b(0,0,0);
    }

  for(float x = 0; x<wu; x+=1)
    for(float y=0;y<hu;y+=1){
      float rx = (x-cxu);
      float ry = (y-cyu);
      float r = sqrtf(rx*rx+ry*ry);
      float rr = (sc/10.0)*sinf(atanf(r/flx))*flx;
      float xx = cx + rr*rx/r;
      float yy = cy + rr*ry/r;

      if(xx>=0&&yy>=0&&xx<w&&yy<h)
        img_undistorted.at<cv::Vec3b>((int)y,(int)x) = img.at<cv::Vec3b>((int)yy,(int)xx);
    }

  cv::Point2f t1(-1, 0);
  cv::Point2f t2(1, 0);
  cv::Point2f t3(0, -1);
  cv::Point2f t4(0, 1);
  printf("=============================================\n");
  printf("c: %.2f,%.2f -> i: %.2f,%.2f -> c: %.2f,%.2f\n",
      t1.x, t1.y, c2i(t1).x, c2i(t1).y, i2c(c2i(t1)).x, i2c(c2i(t1)).y);
  printf("c: %.2f,%.2f -> i: %.2f,%.2f -> c: %.2f,%.2f\n",
      t2.x, t2.y, c2i(t2).x, c2i(t2).y, i2c(c2i(t2)).x, i2c(c2i(t2)).y);
  printf("c: %.2f,%.2f -> i: %.2f,%.2f -> c: %.2f,%.2f\n",
      t3.x, t3.y, c2i(t3).x, c2i(t3).y, i2c(c2i(t3)).x, i2c(c2i(t3)).y);
    printf("c: %.2f,%.2f -> i: %.2f,%.2f -> c: %.2f,%.2f\n",
        t4.x, t4.y, c2i(t4).x, c2i(t4).y, i2c(c2i(t4)).x, i2c(c2i(t4)).y);

    cv::line(img_undistorted, c2i(t1), c2i(t2), cv::Scalar(0, 0, 255), 2);
    cv::line(img_undistorted, c2i(t3), c2i(t4), cv::Scalar(0, 0, 255), 2);

    cv::imshow("undistorted", img_undistorted);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "calibration");

  img = cv::imread("/home/juauer/Schreibtisch/modelcars/captures/(200,200,0).jpg");

  w = img.cols;
  h = img.rows;
  cx = w / 2;
  cy = h / 2;

  cv::namedWindow("undistorted", CV_WINDOW_AUTOSIZE);
  cv::createTrackbar("flx", "undistorted", &flx, flx_max, apply);
  cv::createTrackbar("h (cm)", "undistorted", &ch, ch_max, apply);
  cv::createTrackbar("scale", "undistorted", &sc, sc_max, apply);
  apply(0, (void*)0);
  cv::waitKey(0);
  return 0;
}

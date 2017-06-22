#include <stdlib.h>
#include <math.h>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <ros/package.h>
#include <ros/console.h>

cv::Scalar colors[] {
    cv::Scalar(255,   0,   0),
    cv::Scalar(  0, 255,   0),
    cv::Scalar(  0,   0, 255),
    cv::Scalar(127, 127,   0),
    cv::Scalar(127,   0, 127),
    cv::Scalar(  0, 127, 127),
    cv::Scalar( 85,  85,  85),
    cv::Scalar( 85, 170,   0),
    cv::Scalar( 85,   0, 170),
    cv::Scalar(  0,  85, 170),
    cv::Scalar(170,  85,   0),
    cv::Scalar(170,   0,  85),
    cv::Scalar(  0, 170,  85)
};

int cs = 13;
int c = 0;

cv::Scalar nextCol() {
  c = (c + 1) % cs;
  return colors[c];
}

// e.g. 'rosrun test trajectory_plotter 900 track01 track02 track03'
int main(int argc, char **argv) {
  int n = atoi(argv[1]);

  std::vector<std::string> logs;

  for(int i = 2; i < argc; ++i)
    logs.push_back(ros::package::getPath("test")
        + std::string("/../../../logs/") + std::string(argv[i]) );

  float meanx[n];
  float meany[n];

  for(int i = 0; i < n; ++i) {
    meanx[i] = 0;
    meany[i] = 0;
  }

  cv::Mat img(540, 720, CV_8UC3, cv::Scalar(255, 255, 255) );
  float vx[argc - 2][n];
  float vy[argc - 2][n];
  int j    = 0;
  float ji = 1.0 / (argc - 2);

  for(std::vector<std::string>::const_iterator it = logs.begin();
      it != logs.end(); ++it) {

    std::ifstream file;
    std::string line_str;
    float x, y;
    cv::Point2f last;
    bool b = true;
    int i  = 0;
    file.open(it->c_str(), std::ios::in);

    while(i < n && getline(file, line_str) ) {
      std::stringstream line_ss(line_str);
      line_ss >> x >> y;
      x = 100 * x + 360;
      y = 100 * y + 270;
      vx[j][i]  = x;
      vy[j][i]  = y;
      meanx[i] += ji * x;
      meany[i] += ji * y;

      if(b) {
        last = cv::Point2f(x, y);
        b = false;
        continue;
      }

      cv::Point2f now(x, y);
      cv::line(img, now, last, nextCol() );
      last = cv::Point2f(x, y);
      ++i;
    }

    ++j;
    file.close();
  }

  float stddev = 0;
  int lb = (int)floor(0.05*n);
  int ub = (int)ceil(0.95*n);
  j = 0;

  for(int k = 0; k < argc - 2; ++k)
    for(int i = lb; i < ub; ++i) {
      float x = vx[k][i] - meanx[i];
      float y = vy[k][i] - meany[i];
      stddev += sqrtf(x*x + y*y);
    }

  ROS_INFO("%f", stddev / ( (argc - 2) * (ub - lb) ) );
  std::string path = ros::package::getPath("test") + std::string("/../../../logs/test.png");
  cv::imshow("tracks", img);
  cv::imwrite(path.c_str(), img);
  cv::waitKey(0);
}

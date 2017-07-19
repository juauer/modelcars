#include <stdlib.h>
#include <math.h>
#include <limits>
#include <string>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/package.h>
#include <ros/console.h>

// some rgb colors
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

const int cs = sizeof(colors) / sizeof(cv::Scalar);
int c        = 0;

cv::Scalar nextCol() {
  c = (c + 1) % cs;
  return colors[c];
}

/**
 * Use with 'rosrun cps2 trajectory_plotter FILE... '
 * Plot the trajectories given by filenames FILEs (relative to catkin_ws/logs/).
 *
 * All logs should have been recorded using the same bagfile (otherwise the computed
 * stddev won't yield any useful information).
 *
 * It looks like we're not allowed to parse positional args with roslaunch, yet.
 * That's why there is no launchfile.
 */
int main(int argc, char **argv) {
  // read in all logs, count lines and ranges of x, y
  std::vector<cv::Point2f> values[argc - 1];

  int n       = std::numeric_limits<int>::max();
  float min_x = std::numeric_limits<float>::max();
  float min_y = std::numeric_limits<float>::max();
  float max_x = std::numeric_limits<float>::min();
  float max_y = std::numeric_limits<float>::min();

  for(int i = 1; i < argc; ++i) {
    std::string path = ros::package::getPath("cps2")
          + std::string("/../../../logs/") + std::string(argv[i]);

    int n_line = 0;

    std::ifstream file;
    std::string line_str;
    float x, y;
    file.open(path.c_str(), std::ios::in);

    while(getline(file, line_str) ) {
      std::stringstream line_ss(line_str);
      line_ss >> x >> y;
      values[i - 1].push_back(cv::Point2f(x, y) );

      min_x = std::min(min_x, x);
      min_y = std::min(min_y, y);
      max_x = std::max(max_x, x);
      max_y = std::max(max_y, y);
      ++n_line;
    }

    n = std::min(n, n_line);

    file.close();
  }

  // compute means
  cv::Point2f means[n];

  float ni = 1.0 / (argc - 1);

  for(int i = 0; i < argc - 1; ++i)
    for(int j = 0; j < n; ++j)
      means[j] += ni * values[i].at(j);

  // compute standard deviation
  float stddev = 0;

  for(int i = 0; i < argc - 1; ++i)
    for(int j = 0; j < n; ++j) {
      const float x = values[i].at(j).x - means[j].x;
      const float y = values[i].at(j).y - means[j].y;

      stddev += sqrtf(x*x + y*y);
    }

  stddev /= n * (argc - 1);

  // plot tracks
  cv::Mat img(
      (int)ceilf(100 * (max_y - min_y) + 40),
      (int)ceilf(100 * (max_x - min_x) + 40),
      CV_8UC3, cv::Scalar(255, 255, 255) );

  for(int i = 0; i < argc - 1; ++i) {
    cv::Scalar col   = nextCol();
    cv::Point2f last = values[i].front();

    for(int j = 0; j< n; ++j) {
      cv::line(img,
          cv::Point2i(
              (int)roundf(100 * (last.x - min_x) + 20),
              (int)roundf(100 * (last.y - min_y) + 20) ),
          cv::Point2i(
              (int)roundf(100 * (values[i].at(j).x - min_x) + 20 ),
              (int)roundf(100 * (values[i].at(j).y - min_y) + 20) ),
          col);

      last = values[i].at(j);
    }
  }

  // output results
  ROS_INFO("stddev: %.2f m", stddev);
  cv::imshow("tracks", img);
  cv::imwrite( (ros::package::getPath("cps2")
      + std::string("/../../../logs/test.png") ).c_str(), img);
  cv::waitKey(0);
}

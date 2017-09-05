#include <stdlib.h>
#include <math.h>
#include <limits>
#include <string>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/console.h>
#include <ros/package.h>

// scaling factor to convert meters to pixels
const float m2p = 90.0;

// keyboard control sensitivity
const float kb_step = 0.05;

// some bgr colors
cv::Scalar colors[] {
    cv::Scalar(255, 255,   0),
    cv::Scalar(255,   0, 255),
    cv::Scalar(  0, 255, 255),
    cv::Scalar(127, 255,   0),
    cv::Scalar(127,   0, 255),
    cv::Scalar(  0, 127, 255),
    cv::Scalar(255, 127,   0),
    cv::Scalar(255,   0, 127),
    cv::Scalar(  0, 255, 127),
    cv::Scalar(255,   0,   0),
    cv::Scalar(  0, 255,   0),
    cv::Scalar(  0,   0, 255)
};

const int cs = sizeof(colors) / sizeof(cv::Scalar);
int c        = -1;

cv::Scalar nextCol() {
  c = (c + 1) % cs;
  return colors[c];
}

/**
 * representation for a single logfile
 */
class Track {
public:
  std::string path;
  std::vector<cv::Point2f> values;
  cv::Point2f center;
  cv::Point3f offset;

  const cv::Scalar color = nextCol();

  Track() {}

  ~Track() {}

  void init(const std::string _path) {
    // read in the logfile
    path = _path;

    std::ifstream file;
    std::string line_str;
    float x, y;
    file.open(path.c_str(), std::ios::in);

    // first line should contain offsets
    getline(file, line_str);
    std::stringstream offset_ss(line_str);
    offset_ss >> offset.x >> offset.y >> offset.z;

    while(getline(file, line_str) ) {
      std::stringstream line_ss(line_str);
      line_ss >> x >> y;
      values.push_back(cv::Point2f(x, y) );
    }

    file.close();

    // compute center
    cv::Point2f mins = values.front();
    cv::Point2f maxs = values.front();

    for(std::vector<cv::Point2f>::const_iterator i = values.begin(); i != values.end(); ++i) {
      if(i->x < mins.x)
        mins.x = i->x;
      else if(i->x > maxs.x)
        maxs.x = i->x;

      if(i->y < mins.y)
        mins.y = i->y;
      else if(i->y > maxs.y)
        maxs.y = i->y;
    }

    center = mins + (maxs - mins) / 2;
  }

  cv::Point2f operator [](int i) {
    cv::Point2f vec = values[i] - center;

    return cv::Point2f(
        vec.x * cos(offset.z) - vec.y * sin(offset.z) + center.x + offset.x,
        vec.x * sin(offset.z) + vec.y * cos(offset.z) + center.y + offset.y
    );
  }
};

// number of tracks
int n_tracks;

// number of samples to use
int n_samples;

// all tracks
Track *tracks;

// image and background image
cv::Mat img, bg;

/**
 * plot all tracks
 */
void repaint() {
  bg.copyTo(img);

  for(int i = 0; i < n_tracks; ++i) {
    cv::Point2f last = tracks[i][0];

    for(int j = 0; j < n_samples; ++j) {
      cv::line(img,
          cv::Point2i(
              (int)roundf(m2p * last.x),
              (int)roundf(m2p * last.y) ),
          cv::Point2i(
              (int)roundf(m2p * tracks[i][j].x),
              (int)roundf(m2p * tracks[i][j].y) ),
          tracks[i].color);

      last = tracks[i][j];
    }
  }

  cv::imshow("tracks", img);
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
  // use an image of the labs floor as background
  bg = cv::imread( (ros::package::getPath("cps2")
      + std::string("/../../../captures/ground_map_lab.png") ).c_str() );

  // read in all logs and count the number samples
  n_tracks  = argc - 1;
  n_samples = std::numeric_limits<int>::max();
  tracks    = new Track[n_tracks];

  for(int i = 0; i < n_tracks; ++i) {
    tracks[i].init(ros::package::getPath("cps2") + std::string("/../../../logs/")
        + std::string(argv[i + 1]) );

    n_samples = std::min(n_samples, (int)tracks[i].values.size() );
  }

  // compute means
  cv::Point2f means[n_samples];

  float ni = 1.0 / n_tracks;

  for(int i = 0; i < n_tracks; ++i)
    for(int j = 0; j < n_samples; ++j)
      means[j] += ni * tracks[i][j];

  // compute standard deviation
  float stddev = 0;

  for(int i = 0; i < n_tracks; ++i)
    for(int j = 0; j < n_samples; ++j) {
      const float x = tracks[i][j].x - means[j].x;
      const float y = tracks[i][j].y - means[j].y;

      stddev += sqrtf(x*x + y*y);
    }

  stddev /= n_tracks * n_samples;

  ROS_INFO("stddev: %.2f m", stddev);

  // let the user adjust offsets with the keyboard for some adorable plots
  int c;

  bool running = true;
  int track    = 0;

  ROS_INFO("use [,.] to cycle through the tracks.");
  ROS_INFO("use [wasdqe] to change the offsets of the selected track.");
  ROS_INFO("Hit RETURN to quit.");
  ROS_INFO("Selected track: %d", track);
  repaint();

  while(running) {
    c = cv::waitKey();

    switch(c) {
    case 44:  // ,
      if(track == 0)
        track = n_tracks - 1;
      else
        --track;

      ROS_INFO("Selected track: %d", track);
      break;

    case 46:  // .
      if(track == n_tracks - 1)
        track = 0;
      else
        ++track;

      ROS_INFO("Selected track: %d", track);
      break;

    case 113: // q
      tracks[track].offset.z -= kb_step;
      break;

    case 119: // w
      tracks[track].offset.y -= kb_step;
      break;

    case 101: // e
      tracks[track].offset.z += kb_step;
      break;

    case 97:  // a
      tracks[track].offset.x -= kb_step;
      break;

    case 115: // s
      tracks[track].offset.y += kb_step;
      break;

    case 100: // d
      tracks[track].offset.x += kb_step;
      break;

    case 13: // return
      running = false;
      break;

    default:
      break;
    }

    repaint();
  }

  // save the image and write back the logfiles with changed offsets
  cv::imwrite( (ros::package::getPath("cps2")
      + std::string("/../../../logs/test.png") ).c_str(), img);

  ROS_INFO("Image written to logs/test.png");

  for(int i = 0; i < n_tracks; ++i) {
    std::ofstream file;
    file.open(tracks[i].path.c_str(), std::ios::out | std::ios::trunc);
    file << tracks[i].offset.x << " " << tracks[i].offset.y << " " << tracks[i].offset.z << std::endl;

    for(std::vector<cv::Point2f>::const_iterator j = tracks[i].values.begin();
        j != tracks[i].values.end(); ++j)
      file << j->x << " " << j->y << std::endl;

    file.close();
  }
}

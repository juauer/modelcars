#ifndef SRC_MAP_PIECE_HPP_
#define SRC_MAP_PIECE_HPP_

#include <opencv2/core/core.hpp>
#include <ros/time.h>

namespace cps2 {

class MapPiece {
public:
  MapPiece() :
    is_set(false),
    pos_world(cv::Point3f() ),
    img(cv::Mat(0, 0, CV_8UC1) ),
    stamp(ros::Time() ) {}

  virtual ~MapPiece() {}

  bool is_set;
  cv::Point3f pos_world;
  cv::Mat img;
  ros::Time stamp;
};
}

#endif /* SRC_MAP_PIECE_HPP_ */

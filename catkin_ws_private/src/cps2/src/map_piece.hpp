#ifndef SRC_MAP_PIECE_HPP_
#define SRC_MAP_PIECE_HPP_

#include <opencv2/core.hpp>

namespace cps2 {

class MapPiece {
public:
  MapPiece() {}
  MapPiece(cv::Point3f &_pos_world, cv::Mat &_img) :
    pos_world(_pos_world),
    img(_img) {}

  virtual ~MapPiece() {}

  cv::Point3f pos_world;
  cv::Mat img;
};
}

#endif /* SRC_MAP_PIECE_HPP_ */

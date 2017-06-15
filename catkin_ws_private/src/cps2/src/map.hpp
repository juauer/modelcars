#ifndef SRC_MAP_HPP_
#define SRC_MAP_HPP_

#include <vector>
#include <ros/ros.h>
#include <opencv2/core.hpp>
#include "fisheye_camera_matrix/camera_matrix.hpp"
#include "image_evaluator.hpp"
#include "map_piece.hpp"

namespace cps2 {

class Map {
public:
  Map(const char *path, cps2::ImageEvaluator *image_evaluator);
  virtual ~Map();

  std::vector<cv::Mat> get_map_pieces(const cv::Point3f &pos_world);
  void update(const cv::Point3f &pos_world_last, const cv::Point3f &pos_world_now,
      const cv::Point2f &odom, float belief_best,
      const fisheye_camera_matrix::CameraMatrix &camera_matrix);

  cps2::MapPiece theOnePiece;
  cv::Rect2f bbox;

private:
  bool ready;
  cps2::ImageEvaluator *image_evaluator;
  fisheye_camera_matrix::CameraMatrix camera_matrix;
  cv::Point3f origin;
};

} /* namespace cps2 */

#endif /* SRC_MAP_HPP_ */

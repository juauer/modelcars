#include <math.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "map.hpp"

namespace cps2 {

Map::Map(float _grid_size, cps2::ImageEvaluator *_image_evaluator)
    : grid_size(_grid_size),
      ready(false),
      image_evaluator(_image_evaluator)
{}

Map::~Map() {

}

std::vector<cv::Mat> Map::get_map_pieces(const cv::Point3f &pos_world) {
  std::vector<cv::Mat> map_pieces;

  if(!ready)
    return map_pieces;

  // TODO locate and push_back several images near pos_world

  cv::Point2f pos_rel2f(pos_world.x, pos_world.y);

  cv::Point2i pos_image = camera_matrix.relative2image(pos_rel2f);

  cv::Point3f pos_image3f(pos_image.x, pos_image.y, pos_world.z);

  map_pieces.push_back(image_evaluator->transform(
      theOnePiece.img, pos_image3f, cv::Size2i(camera_matrix.width, camera_matrix.height) ) );

  return map_pieces;
}

void Map::update(const cv::Mat &image, const Particle &pos_world,
      const fisheye_camera_matrix::CameraMatrix &_camera_matrix) {

  // TODO get rid of the onePieceMapHACK

  fisheye_camera_matrix::CameraMatrix onePieceHackedCM(
      _camera_matrix.width, _camera_matrix.height,
      theOnePiece.img.cols / 2, theOnePiece.img.rows / 2,
      _camera_matrix.fl, _camera_matrix.ceil_height, _camera_matrix.scale);

  camera_matrix = onePieceHackedCM;

  // TODO save image at coordinates pos_world and update bbox (not every frame)

  cv::Point2i p_img(0, 0);

  cv::Point2f p_rel = camera_matrix.image2relative(p_img);
  cv::Point2f p_abs = cv::Point2f(fabs(p_rel.x), fabs(p_rel.y) );

  // TODO do something smart to handle the borders instead of using magic numbers

  bbox.x      = -0.9 * p_abs.x;
  bbox.y      = -0.9 * p_abs.y;
  bbox.width  = 1.8 * p_abs.x;
  bbox.height = 1.8 * p_abs.y;
  ready       = true;
}

inline void Map::world2grid(const cv::Point3f &pos_world, int &grid_x, int &grid_y) {
  grid_x = (int)floorf( (pos_world.x - bbox.x) / grid_size);
  grid_y = (int)floorf( (pos_world.y - bbox.y) / grid_size);
}


inline cv::Point2f Map::grid2world(const int &grid_x, const int &grid_y) {
  return cv::Point2f(
      grid_x * grid_size - bbox.x + grid_size / 2,
      grid_y * grid_size - bbox.y + grid_size / 2
  );
}

}
